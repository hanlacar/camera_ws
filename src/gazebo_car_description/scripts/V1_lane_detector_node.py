#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32, String


def rosimg_to_bgr(msg: Image) -> np.ndarray:
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        raise RuntimeError('invalid image size')

    enc = (msg.encoding or '').lower()
    step = int(msg.step) if msg.step else 0
    data = np.frombuffer(msg.data, dtype=np.uint8)

    def require(nbytes: int) -> None:
        if data.size < nbytes:
            raise RuntimeError(f'buffer too small: size={data.size}, need>={nbytes}')

    if enc == 'bgr8':
        row_bytes = w * 3
        s = step if step else row_bytes
        require(h * s)
        rows = data[: h * s].reshape((h, s))[:, :row_bytes]
        return rows.reshape((h, w, 3)).copy()

    if enc == 'rgb8':
        row_bytes = w * 3
        s = step if step else row_bytes
        require(h * s)
        rows = data[: h * s].reshape((h, s))[:, :row_bytes]
        rgb = rows.reshape((h, w, 3)).copy()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if enc in ('mono8', '8uc1'):
        row_bytes = w
        s = step if step else row_bytes
        require(h * s)
        rows = data[: h * s].reshape((h, s))[:, :row_bytes]
        gray = rows.reshape((h, w)).copy()
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if enc in ('yuv422_yuy2', 'yuyv', 'yuy2'):
        row_bytes = w * 2
        s = step if step else row_bytes
        require(h * s)
        rows = data[: h * s].reshape((h, s))[:, :row_bytes]
        yuyv = rows.reshape((h, w, 2)).copy()
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)

    if enc in ('nv12', 'nv21'):
        s = step if step else w
        if s < w:
            raise RuntimeError(f'{enc} step({s}) < width({w})')
        y_bytes = h * s
        uv_h = h // 2
        need = y_bytes + uv_h * s
        require(need)
        y = data[:y_bytes].reshape((h, s))[:, :w].copy()
        uv = data[y_bytes:y_bytes + uv_h * s].reshape((uv_h, s))[:, :w].copy()
        yuv = np.vstack((y, uv))
        code = cv2.COLOR_YUV2BGR_NV12 if enc == 'nv12' else cv2.COLOR_YUV2BGR_NV21
        return cv2.cvtColor(yuv, code)

    raise RuntimeError(f'unsupported encoding: {msg.encoding}')


def bgr_to_imgmsg(bgr: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(bgr.shape[0])
    msg.width = int(bgr.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = int(bgr.shape[1] * 3)
    msg.data = bgr.tobytes()
    return msg


class LaneDetector(Node):
    def __init__(self) -> None:
        super().__init__('lane_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/lane/debug_image')
        self.declare_parameter('min_main_separation_px', 60)
        self.declare_parameter('lane_half_width_px', 85.0)
        self.declare_parameter('reference_row_ratio', 0.82)

        self.ROI_START_RATIO = 0.50

        self.YELLOW_H_LOW = 15
        self.YELLOW_H_HIGH = 65
        self.YELLOW_S_LOW = 40
        self.YELLOW_V_LOW = 60

        self.WHITE_V_LOW = 150
        self.WHITE_S_HIGH = 90

        self.MORPH_K = 3
        self.CLOSE_ITER = 1
        self.DILATE_ITER = 1

        self.NUM_SLICES = 9
        self.WIN_MARGIN = 60
        self.MIN_M00 = 50.0
        self.MAX_MISS = 2
        self.DRAW_SMOOTH_PATH = True

        self.SIDE_ENABLE = True
        self.CORRIDOR_MARGIN_IN = 4
        self.MAIN_GUARD_PX = 25

        self.CLAHE_CLIP = 2.0
        self.CLAHE_TILE = 8

        self.OTSU_MIN_AREA = 30
        self.OTSU_MAX_AREA_RATIO = 0.35

        self.FAR_HOLD_FRAMES = 8
        self.YELLOW_MIN_PIX_BOTTOM = 30
        self.last_far_dir = 0
        self.lost_count = 0

        self.clahe = cv2.createCLAHE(
            clipLimit=float(self.CLAHE_CLIP),
            tileGridSize=(int(self.CLAHE_TILE), int(self.CLAHE_TILE)),
        )

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.lane_half_width_px = float(self.get_parameter('lane_half_width_px').value)
        self.reference_row_ratio = float(self.get_parameter('reference_row_ratio').value)
        self.min_main_separation_px = int(self.get_parameter('min_main_separation_px').value)

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(Image, self.image_topic, self.cb, qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Image, self.output_topic, pub_qos)

        self.detected_pub = self.create_publisher(Bool, '/lane/detected', 10)
        self.main_x_pub = self.create_publisher(Float32, '/lane/main_x_px', 10)
        self.side_x_pub = self.create_publisher(Float32, '/lane/side_x_px', 10)
        self.center_x_pub = self.create_publisher(Float32, '/lane/center_x_px', 10)
        self.error_px_pub = self.create_publisher(Float32, '/lane/error_px', 10)
        self.error_norm_pub = self.create_publisher(Float32, '/lane/error_norm', 10)
        self.roi_y0_pub = self.create_publisher(Int32, '/lane/roi_y0_px', 10)
        self.image_center_pub = self.create_publisher(Float32, '/lane/image_center_x_px', 10)
        self.mode_pub = self.create_publisher(String, '/lane/mode', 10)
        self.confidence_pub = self.create_publisher(Float32, '/lane/confidence', 10)

        self.get_logger().info(
            f'lane_detector started | image_topic={self.image_topic} output_topic={self.output_topic}'
        )

    @staticmethod
    def get_contour_centroids(binary_slice: np.ndarray, min_m00: float):
        contours, _ = cv2.findContours(binary_slice, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        xs = []
        for contour in contours:
            moments = cv2.moments(contour)
            if moments['m00'] < min_m00:
                continue
            cx = int(moments['m10'] / (moments['m00'] + 1.0e-6))
            xs.append(cx)
        return xs

    @staticmethod
    def track_from_seed(
        out_img: np.ndarray,
        mask: np.ndarray,
        y0: int,
        seed_x: int,
        num_slices: int,
        slice_h: int,
        margin: int,
        min_m00: float,
        max_miss: int,
        draw_smooth: bool,
        color_path=(0, 255, 0),
        draw_boxes=True,
        start_i=0,
    ):
        height, width = out_img.shape[:2]
        roi_h, roi_w = mask.shape[:2]

        lane_x = []
        lane_y = []
        current_x = int(seed_x)
        miss_count = 0

        ys_init = roi_h - (start_i + 1) * slice_h
        ye_init = roi_h - start_i * slice_h
        ys_init = max(0, ys_init)
        ye_init = max(ys_init + 1, ye_init)

        if draw_boxes:
            cv2.rectangle(
                out_img,
                (max(0, current_x - margin), y0 + ys_init),
                (min(roi_w - 1, current_x + margin), y0 + ye_init),
                (0, 0, 0),
                1,
            )

        lane_x.append(current_x)
        lane_y.append(int(ys_init + slice_h / 2))

        for i in range(start_i + 1, num_slices):
            ys = roi_h - (i + 1) * slice_h
            ye = roi_h - i * slice_h
            if ye <= 0:
                break
            ys = max(0, ys)
            ye = max(ys + 1, ye)

            x_min = max(0, current_x - margin)
            x_max = min(roi_w, current_x + margin)
            slice_mask = mask[ys:ye, x_min:x_max]
            contours, _ = cv2.findContours(slice_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            valid_found = False
            if contours:
                contour = max(contours, key=cv2.contourArea)
                moments = cv2.moments(contour)
                if moments['m00'] >= min_m00:
                    valid_found = True
                    cx_local = int(moments['m10'] / (moments['m00'] + 1.0e-6))
                    current_x = cx_local + x_min
                    lane_x.append(current_x)
                    lane_y.append(int(ys + slice_h / 2))

            if draw_boxes:
                cv2.rectangle(
                    out_img,
                    (max(0, current_x - margin), y0 + ys),
                    (min(roi_w - 1, current_x + margin), y0 + ye),
                    (0, 0, 0),
                    1,
                )

            if not valid_found:
                miss_count += 1
                if miss_count > max_miss:
                    break
            else:
                miss_count = 0

        start_pt = (width // 2, height - 1)
        fit = None

        if len(lane_x) >= 3 and draw_smooth:
            fit = np.polyfit(lane_y, lane_x, 2)
            y_top = int(min(lane_y))
            ys2 = np.arange(roi_h - 1, y_top - 1, -5, dtype=np.int32)
            curve = [list(start_pt)]
            for yy in ys2:
                xx = int(fit[0] * yy * yy + fit[1] * yy + fit[2])
                xx = max(0, min(roi_w - 1, xx))
                curve.append([xx, y0 + int(yy)])
            curve = np.array(curve, dtype=np.int32)
            if curve.shape[0] >= 2:
                cv2.polylines(out_img, [curve.reshape(-1, 1, 2)], False, color_path, 2)
        else:
            points = [list(start_pt)]
            for lane_px, lane_py in zip(lane_x, lane_y):
                points.append([int(lane_px), int(y0 + lane_py)])
            points = np.array(points, dtype=np.int32)
            if points.shape[0] >= 2:
                cv2.polylines(out_img, [points.reshape(-1, 1, 2)], False, color_path, 2)

        return lane_x, lane_y, fit

    @staticmethod
    def build_corridor_mask_dir(ymask_roi, fit_main, far_dir: int, margin_in=4, step_y=2, main_guard=25):
        roi_h, roi_w = ymask_roi.shape[:2]
        corridor = np.zeros_like(ymask_roi, dtype=np.uint8)
        last_yx = None

        for yy in range(0, roi_h, step_y):
            xm = int(fit_main[0] * yy * yy + fit_main[1] * yy + fit_main[2])
            xm = max(0, min(roi_w - 1, xm))

            xs = np.where(ymask_roi[yy, :] > 0)[0]
            if xs.size == 0:
                if last_yx is None:
                    continue
                yx = last_yx
            else:
                yx = int(xs.min() if far_dir < 0 else xs.max())
                last_yx = yx

            if yx > xm:
                lo = xm + main_guard + margin_in
                hi = yx - margin_in
            else:
                lo = yx + margin_in
                hi = xm - main_guard - margin_in

            lo = max(0, lo)
            hi = min(roi_w - 1, hi)
            if hi - lo < 10:
                continue

            corridor[yy:yy + step_y, lo:hi] = 255

        return corridor

    @staticmethod
    def pick_far_yellow_x_from_bottom(ymask: np.ndarray, y_start0: int, y_end0: int, far_dir: int) -> int:
        roi_w = ymask.shape[1]
        xs_y = np.where(ymask[y_start0:y_end0, :] > 0)[1]
        if xs_y.size > 0:
            return int(xs_y.min() if far_dir < 0 else xs_y.max())
        return 0 if far_dir < 0 else (roi_w - 1)

    def estimate_far_dir_bottom(self, ymask: np.ndarray, y_start0: int, y_end0: int, xm_ref: int):
        xs = np.where(ymask[y_start0:y_end0, :] > 0)[1]
        if xs.size >= int(self.YELLOW_MIN_PIX_BOTTOM):
            x_left = int(np.quantile(xs, 0.10))
            x_right = int(np.quantile(xs, 0.90))
            d_left = abs(xm_ref - x_left)
            d_right = abs(xm_ref - x_right)
            far_dir = -1 if d_left > d_right else +1
            self.last_far_dir = far_dir
            self.lost_count = 0
            return far_dir

        self.lost_count += 1
        if self.lost_count <= int(self.FAR_HOLD_FRAMES) and self.last_far_dir != 0:
            return self.last_far_dir
        return 0

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _finite_or_nan(value):
        return value if value is not None and math.isfinite(value) else float('nan')

    def _publish_float(self, publisher, value) -> None:
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def _publish_int(self, publisher, value: int) -> None:
        msg = Int32()
        msg.data = int(value)
        publisher.publish(msg)

    def _publish_bool(self, publisher, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def _publish_string(self, publisher, value: str) -> None:
        msg = String()
        msg.data = value
        publisher.publish(msg)

    @staticmethod
    def _evaluate_lane_x(fit, lane_x, reference_y, width: int):
        if fit is not None:
            x_val = float(fit[0] * reference_y * reference_y + fit[1] * reference_y + fit[2])
            return float(max(0.0, min(width - 1.0, x_val)))
        if lane_x:
            return float(max(0.0, min(width - 1.0, lane_x[0])))
        return float('nan')

    def _fallback_center_from_single_lane(self, lane_x: float, image_center_x: float, side_hint: int) -> float:
        if side_hint != 0:
            return lane_x + float(side_hint) * self.lane_half_width_px
        if lane_x < image_center_x:
            return lane_x + self.lane_half_width_px
        return lane_x - self.lane_half_width_px

    def _draw_metrics(
        self,
        image: np.ndarray,
        detected: bool,
        mode: str,
        confidence: float,
        main_x: float,
        side_x: float,
        center_x: float,
        image_center_x: float,
        error_px: float,
        error_norm: float,
        roi_y0: int,
    ) -> None:
        height, width = image.shape[:2]
        color_main = (0, 255, 0)
        color_side = (255, 255, 0)
        color_center = (255, 0, 255)
        color_image_center = (0, 255, 255)

        for x_value, color in (
            (main_x, color_main),
            (side_x, color_side),
            (center_x, color_center),
            (image_center_x, color_image_center),
        ):
            if math.isfinite(x_value):
                xx = int(self._clamp(x_value, 0, width - 1))
                cv2.line(image, (xx, roi_y0), (xx, height - 1), color, 2)

        cv2.putText(
            image,
            f'detected={detected} mode={mode} conf={confidence:.2f}',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            image,
            f'main={main_x:.1f} side={side_x:.1f} center={center_x:.1f}',
            (15, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            image,
            f'img_center={image_center_x:.1f} err_px={error_px:.1f} err_norm={error_norm:.3f}',
            (15, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

    def _publish_measurements(
        self,
        detected: bool,
        main_x: float,
        side_x: float,
        center_x: float,
        error_px: float,
        error_norm: float,
        roi_y0: int,
        image_center_x: float,
        mode: str,
        confidence: float,
    ) -> None:
        self._publish_bool(self.detected_pub, detected)
        self._publish_float(self.main_x_pub, self._finite_or_nan(main_x))
        self._publish_float(self.side_x_pub, self._finite_or_nan(side_x))
        self._publish_float(self.center_x_pub, self._finite_or_nan(center_x))
        self._publish_float(self.error_px_pub, self._finite_or_nan(error_px))
        self._publish_float(self.error_norm_pub, self._finite_or_nan(error_norm))
        self._publish_int(self.roi_y0_pub, roi_y0)
        self._publish_float(self.image_center_pub, image_center_x)
        self._publish_string(self.mode_pub, mode)
        self._publish_float(self.confidence_pub, confidence)

    def cb(self, msg: Image) -> None:
        try:
            frame = rosimg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f'decode fail: encoding={msg.encoding}, err={exc}')
            return

        height, width = frame.shape[:2]
        y0 = int(height * self.ROI_START_RATIO)
        y0 = max(0, min(height - 1, y0))

        roi = frame[y0:height, :]
        roi_h, roi_w = roi.shape[:2]
        out = frame.copy()
        cv2.rectangle(out, (0, y0), (width - 1, height - 1), (0, 255, 0), 2)

        image_center_x = width * 0.5
        if roi_h < 30:
            self._publish_measurements(
                detected=False,
                main_x=float('nan'),
                side_x=float('nan'),
                center_x=float('nan'),
                error_px=float('nan'),
                error_norm=float('nan'),
                roi_y0=y0,
                image_center_x=image_center_x,
                mode='lost',
                confidence=0.0,
            )
            self.debug_pub.publish(bgr_to_imgmsg(out, msg.header))
            return

        roi_blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2GRAY)

        ymask = cv2.inRange(
            hsv,
            np.array([self.YELLOW_H_LOW, self.YELLOW_S_LOW, self.YELLOW_V_LOW], dtype=np.uint8),
            np.array([self.YELLOW_H_HIGH, 255, 255], dtype=np.uint8),
        )

        gray_clahe = self.clahe.apply(gray)
        _, w_otsu = cv2.threshold(gray_clahe, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        w_hsv = cv2.inRange(
            hsv,
            np.array([0, 0, self.WHITE_V_LOW], dtype=np.uint8),
            np.array([180, self.WHITE_S_HIGH, 255], dtype=np.uint8),
        )
        wmask = cv2.bitwise_and(w_otsu, w_hsv)
        wmask = (wmask > 0).astype(np.uint8) * 255

        num, labels, stats, _ = cv2.connectedComponentsWithStats(wmask, connectivity=8)
        max_area = int(roi_h * roi_w * self.OTSU_MAX_AREA_RATIO)
        min_area = int(self.OTSU_MIN_AREA)
        wmask_filtered = np.zeros_like(wmask)
        for idx in range(1, num):
            area = stats[idx, cv2.CC_STAT_AREA]
            if area < min_area or area > max_area:
                continue
            wmask_filtered[labels == idx] = 255
        wmask = wmask_filtered

        ymask_dilated = cv2.dilate(ymask, np.ones((3, 3), np.uint8), iterations=1)
        wmask = cv2.bitwise_and(wmask, cv2.bitwise_not(ymask_dilated))

        kernel_size = max(3, self.MORPH_K | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        if self.CLOSE_ITER > 0:
            wmask = cv2.morphologyEx(wmask, cv2.MORPH_CLOSE, kernel, iterations=self.CLOSE_ITER)
            ymask = cv2.morphologyEx(ymask, cv2.MORPH_CLOSE, kernel, iterations=1)
        if self.DILATE_ITER > 0:
            wmask = cv2.dilate(wmask, kernel, iterations=self.DILATE_ITER)

        roi_out = out[y0:height, :].copy()
        roi_out[wmask > 0] = (255, 0, 0)
        out[y0:height, :] = roi_out

        num_slices = int(self.NUM_SLICES)
        slice_h = max(8, roi_h // max(1, num_slices))
        margin = int(self.WIN_MARGIN)
        min_m00 = float(self.MIN_M00)
        max_miss = int(self.MAX_MISS)
        draw_smooth = bool(self.DRAW_SMOOTH_PATH)

        y_start0 = max(0, roi_h - slice_h)
        y_end0 = max(y_start0 + 1, roi_h)
        start_white = wmask[y_start0:y_end0, :]

        white_cx_list = self.get_contour_centroids(start_white, min_m00)
        main_seed = None
        if white_cx_list:
            white_cx_list.sort(key=lambda x_val: abs(x_val - image_center_x))
            main_seed = int(white_cx_list[0])

        main_lane_x = []
        fit_main = None
        if main_seed is not None:
            main_lane_x, _, fit_main = self.track_from_seed(
                out_img=out,
                mask=wmask,
                y0=y0,
                seed_x=main_seed,
                num_slices=num_slices,
                slice_h=slice_h,
                margin=margin,
                min_m00=min_m00,
                max_miss=max_miss,
                draw_smooth=draw_smooth,
                color_path=(0, 255, 0),
                draw_boxes=True,
                start_i=0,
            )

        side_lane_x = []
        fit_side = None
        far_dir = 0
        if self.SIDE_ENABLE and fit_main is not None and main_seed is not None:
            xm_ref = int(main_seed)
            far_dir = self.estimate_far_dir_bottom(ymask, y_start0, y_end0, xm_ref)

            if far_dir != 0:
                corridor = self.build_corridor_mask_dir(
                    ymask_roi=ymask,
                    fit_main=fit_main,
                    far_dir=far_dir,
                    margin_in=int(self.CORRIDOR_MARGIN_IN),
                    step_y=2,
                    main_guard=int(self.MAIN_GUARD_PX),
                )
                side_mask = cv2.bitwise_and(wmask, corridor)

                roi_out2 = out[y0:height, :].copy()
                roi_out2[side_mask > 0] = (255, 255, 0)
                out[y0:height, :] = roi_out2

                x_y_far = self.pick_far_yellow_x_from_bottom(ymask, y_start0, y_end0, far_dir)

                side_seed = None
                side_start_idx = 0
                for i in range(num_slices):
                    ys = roi_h - (i + 1) * slice_h
                    ye = roi_h - i * slice_h
                    if ye <= 0:
                        break
                    ys = max(0, ys)
                    ye = max(ys + 1, ye)

                    slice_mask = side_mask[ys:ye, :]
                    cand = self.get_contour_centroids(slice_mask, min_m00)
                    cand_filtered = [
                        cx for cx in cand if abs(cx - xm_ref) >= int(self.min_main_separation_px)
                    ]

                    if cand_filtered:
                        cand_filtered.sort(key=lambda cx: abs(cx - x_y_far))
                        side_seed = int(cand_filtered[0])
                        side_start_idx = i
                        break

                if side_seed is not None:
                    side_lane_x, _, fit_side = self.track_from_seed(
                        out_img=out,
                        mask=side_mask,
                        y0=y0,
                        seed_x=side_seed,
                        num_slices=num_slices,
                        slice_h=slice_h,
                        margin=margin,
                        min_m00=min_m00,
                        max_miss=max_miss,
                        draw_smooth=draw_smooth,
                        color_path=(255, 255, 0),
                        draw_boxes=True,
                        start_i=side_start_idx,
                    )

        contours_y, _ = cv2.findContours(ymask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours_y:
            if cv2.contourArea(contour) < 30:
                continue
            shifted = contour.copy()
            shifted[:, :, 1] += y0
            cv2.drawContours(out, [shifted], -1, (0, 0, 255), 2)

        reference_y_roi = int(self._clamp(self.reference_row_ratio, 0.0, 1.0) * (roi_h - 1))
        main_x = self._evaluate_lane_x(fit_main, main_lane_x, reference_y_roi, roi_w)
        side_x = self._evaluate_lane_x(fit_side, side_lane_x, reference_y_roi, roi_w)

        detected = False
        mode = 'lost'
        confidence = 0.0
        center_x = float('nan')

        if math.isfinite(main_x) and math.isfinite(side_x):
            center_x = 0.5 * (main_x + side_x)
            detected = True
            mode = 'dual'
            confidence = 1.0
        elif math.isfinite(main_x):
            center_x = self._fallback_center_from_single_lane(main_x, image_center_x, far_dir)
            detected = True
            mode = 'main_only'
            confidence = 0.6
        elif math.isfinite(side_x):
            center_x = self._fallback_center_from_single_lane(side_x, image_center_x, 0)
            detected = True
            mode = 'side_only'
            confidence = 0.45

        if detected:
            center_x = self._clamp(center_x, 0.0, width - 1.0)
            error_px = center_x - image_center_x
            error_norm = error_px / (width * 0.5) if width > 0 else float('nan')
        else:
            # Keep the control-side interface alive even when perception is lost.
            center_x = image_center_x
            error_px = 0.0
            error_norm = 0.0

        self._draw_metrics(
            image=out,
            detected=detected,
            mode=mode,
            confidence=confidence,
            main_x=main_x,
            side_x=side_x,
            center_x=center_x,
            image_center_x=image_center_x,
            error_px=error_px,
            error_norm=error_norm,
            roi_y0=y0,
        )
        self._publish_measurements(
            detected=detected,
            main_x=main_x,
            side_x=side_x,
            center_x=center_x,
            error_px=error_px,
            error_norm=error_norm,
            roi_y0=y0,
            image_center_x=image_center_x,
            mode=mode,
            confidence=confidence,
        )
        self.debug_pub.publish(bgr_to_imgmsg(out, msg.header))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
