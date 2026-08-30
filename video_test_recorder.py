#!/usr/bin/env python3
"""Record one independent camera-planner validation run and its raw metrics."""

import csv
import json
import os
import subprocess
import time
from collections import Counter, defaultdict

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, String
from race_interfaces.msg import ImagePath, SemanticPathFrame


def stamp_ns(message):
    return (int(message.header.stamp.sec)*1_000_000_000+
            int(message.header.stamp.nanosec))


class ValidationRecorder(Node):
    def __init__(self):
        super().__init__("camera_validation_recorder")
        for name, value in (("mode", "non_bev"), ("output_dir", "validation"),
                            ("duration_sec", 25.0), ("record_fps", 30.0)):
            self.declare_parameter(name, value)
        self.mode = str(self.get_parameter("mode").value)
        self.output_dir = os.path.abspath(str(self.get_parameter("output_dir").value))
        self.duration = float(self.get_parameter("duration_sec").value)
        self.record_fps = float(self.get_parameter("record_fps").value)
        os.makedirs(self.output_dir, exist_ok=True)
        self.started = time.monotonic()
        self.arrivals = defaultdict(list); self.unique = defaultdict(set)
        self.states = Counter(); self.empty_paths = 0; self.last_metrics = {}
        self.upstream_metrics = {}
        self.current_state = None
        self.current_state_stamp_ns = None
        self.drive_values = []
        self.wheel_values = []
        self.arrival_order_invalid_drive_nonzero = 0
        self.arrival_order_invalid_wheel_nonzero = 0
        self.causal_invalid_drive_nonzero = 0
        self.causal_invalid_wheel_nonzero = 0
        self.frame_index_by_stamp = {}
        self.image_frame_index = 0
        self.controller_trace_by_source = {}
        self.gate_rows_by_source = {}
        self.writer = None; self.overlay_count = 0
        self.overlay_topic = ("/camera/path_overlay_image" if self.mode == "non_bev"
                              else "/camera/bev/overlay_image")
        self.video_path = os.path.join(self.output_dir, f"{self.mode}_overlay.mp4")
        self.png_path = os.path.join(
            self.output_dir, f"{self.mode}_overlay_representative.png")
        reliable = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                              reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Image, "/camera/image_raw",
                                 lambda m: self._mark("image_raw", m),
                                 qos_profile_sensor_data)
        self.create_subscription(SemanticPathFrame, "/perception/semantic_path_frame",
                                 lambda m: self._mark("semantic", m), reliable)
        self.create_subscription(Image, self.overlay_topic, self._on_overlay, reliable)
        if self.mode == "non_bev":
            self.create_subscription(ImagePath, "/camera/image_path_typed",
                                     self._on_non_bev_path, reliable)
            self.create_subscription(String, "/camera/image_path_state",
                                     self._on_plain_state, reliable)
            self.create_subscription(String, "/camera/path_realtime_fps",
                                     self._on_metrics, reliable)
            self.create_subscription(String, "/camera/path_metrics",
                                     self._on_path_metrics, reliable)
            self.create_subscription(Image, "/camera/path_debug_image",
                                     lambda m: self._mark("debug", m), reliable)
        else:
            self.create_subscription(Path, "/camera/bev/path", self._on_bev_path, reliable)
            self.create_subscription(String, "/camera/bev/state", self._on_json_state, reliable)
            self.create_subscription(String, "/camera/bev/diagnostics",
                                     self._on_diagnostics, reliable)
            self.create_subscription(Image, "/camera/bev/overlay",
                                     lambda m: self._mark("debug", m), reliable)
            self.create_subscription(Float32, "/camera_drive",
                                     self._on_drive, reliable)
            self.create_subscription(Int32, "/camera_wheel",
                                     self._on_wheel, reliable)
            self.create_subscription(String, "/camera/bev_drive_diagnostics",
                                     self._on_drive_diagnostics, reliable)
            self.create_subscription(String, "/camera/bev/controller_diagnostics",
                                     self._on_controller_diagnostics, reliable)
        self.create_subscription(String, "/camera/realtime_fps",
                                 self._on_upstream_metrics, reliable)
        self.create_timer(0.2, self._finish_when_due)

    def _mark(self, name, message):
        self.arrivals[name].append(time.monotonic())
        if hasattr(message, "header"):
            stamp = stamp_ns(message)
            self.unique[name].add(stamp)
            if name == "image_raw" and stamp not in self.frame_index_by_stamp:
                self.frame_index_by_stamp[stamp] = self.image_frame_index
                self.image_frame_index += 1

    def _on_overlay(self, message):
        self._mark("overlay", message)
        try:
            raw = np.frombuffer(bytes(message.data), np.uint8)
            image = raw.reshape(message.height, message.step)[:, :message.width*3]
            image = np.ascontiguousarray(image.reshape(message.height, message.width, 3))
            if str(message.encoding).lower() == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if self.writer is None:
                self.writer = cv2.VideoWriter(
                    self.video_path, cv2.VideoWriter_fourcc(*"mp4v"),
                    self.record_fps, (int(message.width), int(message.height)))
                if not self.writer.isOpened():
                    raise RuntimeError("could not open overlay VideoWriter")
            self.writer.write(image); self.overlay_count += 1
            if self.overlay_count == 1 or self.overlay_count % 30 == 0:
                cv2.imwrite(self.png_path, image)
        except Exception as error:
            self.get_logger().error(f"overlay recording failed: {error}")

    def _on_non_bev_path(self, message):
        self._mark("path", message)
        self.empty_paths += int(not message.points)

    def _on_bev_path(self, message):
        self._mark("path", message)
        self.empty_paths += int(not message.poses)
        self._append_jsonl("paths.jsonl", json.dumps({
            "stamp_ns": stamp_ns(message),
            "arrival_monotonic_ns": time.monotonic_ns(),
            "points": [[pose.pose.position.x, pose.pose.position.y]
                       for pose in message.poses],
        }, separators=(",", ":")))

    def _on_plain_state(self, message):
        self.states[str(message.data)] += 1

    def _on_json_state(self, message):
        try:
            value = json.loads(message.data)
            self.current_state = str(value.get("state", value.get("mode", "UNKNOWN")))
            self.current_state_stamp_ns = value.get("stamp_ns")
            self.states[self.current_state] += 1
            self.last_metrics = value
            self._append_jsonl("state_events.jsonl", json.dumps({
                "arrival_monotonic_ns": time.monotonic_ns(),
                "state": self.current_state,
                "stamp_ns": value.get("stamp_ns"),
                "source_stamp_ns": value.get("source_stamp_ns"),
                "reasons": value.get("reasons"),
            }, separators=(",", ":")))
        except (TypeError, ValueError):
            self.states["MALFORMED"] += 1

    def _on_drive(self, message):
        value = float(message.data)
        self.drive_values.append(value)
        if self.current_state == "INVALID" and abs(value) > 1.0e-9:
            self.arrival_order_invalid_drive_nonzero += 1
        self._append_jsonl("command_arrivals.jsonl", json.dumps({
            "arrival_monotonic_ns": time.monotonic_ns(), "topic": "/camera_drive",
            "value": value, "latest_state": self.current_state,
            "latest_state_stamp_ns": self.current_state_stamp_ns,
        }, separators=(",", ":")))

    def _on_wheel(self, message):
        value = int(message.data)
        self.wheel_values.append(value)
        if self.current_state == "INVALID" and value != 0:
            self.arrival_order_invalid_wheel_nonzero += 1
        self._append_jsonl("command_arrivals.jsonl", json.dumps({
            "arrival_monotonic_ns": time.monotonic_ns(), "topic": "/camera_wheel",
            "value": value, "latest_state": self.current_state,
            "latest_state_stamp_ns": self.current_state_stamp_ns,
        }, separators=(",", ":")))

    def _on_drive_diagnostics(self, message):
        self._append_jsonl("drive_diagnostics.jsonl", message.data)
        try:
            value = json.loads(message.data)
            if (value.get("state") == "INVALID" and
                    abs(float(value.get("drive", 0.0))) > 1.0e-9):
                self.causal_invalid_drive_nonzero += 1
        except (TypeError, ValueError):
            pass

    def _on_controller_diagnostics(self, message):
        self._append_jsonl("controller_diagnostics.jsonl", message.data)
        try:
            value = json.loads(message.data)
            if (value.get("planner_state") == "INVALID" and
                    int(value.get("wheel", 0)) != 0):
                self.causal_invalid_wheel_nonzero += 1
            source_stamp = int(value.get("source_stamp_ns") or 0)
            curvature = float(value.get("planner_curvature_per_m") or 0.0)
            if source_stamp > 0 and abs(curvature) > 1.0e-12:
                target = value.get("target_point") or [None, None]
                self.controller_trace_by_source[source_stamp] = {
                    "frame": self.frame_index_by_stamp.get(source_stamp, -1),
                    "source_stamp_ns": source_stamp,
                    "state_stamp_ns": value.get("state_stamp_ns"),
                    "planner_state": value.get("planner_state"),
                    "path_point_count": value.get("path_point_count"),
                    "target_x_m": target[0], "target_y_m": target[1],
                    "lateral_error_m": value.get("lateral_error_m"),
                    "heading_error_deg": value.get("heading_error_deg"),
                    "curvature_per_m": curvature,
                    "target_curvature_per_m": value.get(
                        "target_curvature_per_m"),
                    "atan_wheelbase_curvature_deg": value.get(
                        "bicycle_steering_deg"),
                    "raw_steering_deg": value.get("raw_steering_deg"),
                    "required_steering_deg": value.get(
                        "required_steering_deg"),
                    "filtered_steering_deg": value.get(
                        "temporal_filtered_steering_deg"),
                    "published_camera_wheel": value.get("wheel"),
                }
        except (TypeError, ValueError, IndexError):
            pass

    def _on_metrics(self, message):
        try: self.last_metrics = json.loads(message.data)
        except (TypeError, ValueError): pass

    def _on_upstream_metrics(self, message):
        try: self.upstream_metrics = json.loads(message.data)
        except (TypeError, ValueError): pass

    def _on_path_metrics(self, message):
        with open(os.path.join(self.output_dir, "path_metrics.jsonl"),
                  "a", encoding="utf-8") as stream:
            stream.write(message.data+"\n")

    def _on_diagnostics(self, message):
        self._append_jsonl("diagnostics.jsonl", message.data)
        try:
            value = json.loads(message.data)
            stamp = int(value.get("source_stamp_ns") or value.get("stamp_ns") or 0)
            raw = (value.get("refinement", {}) or {}).get("raw_pixels", {}) or {}
            reasons = value.get("reasons") or []
            self.gate_rows_by_source[stamp] = {
                "frame": self.frame_index_by_stamp.get(stamp, -1),
                "source_stamp": stamp, "road_pixels": value.get("raw_road_pixels"),
                "near_field_coverage": value.get("near_field_coverage"),
                "w_line_pixels": value.get("w_line_pixels", raw.get("white_line", 0)),
                "y_line_pixels": value.get("y_line_pixels", raw.get("yellow_line", 0)),
                "corridor_width_m": value.get("corridor_width_m"),
                "path_point_count": value.get("path_point_count"),
                "path_length_m": value.get("path_length_m"),
                "curvature_per_m": value.get("curvature_per_m"),
                "required_steering_deg": value.get("required_steering_deg"),
                "state": value.get("state"),
                "primary_reason": reasons[0] if reasons else "OK",
                "all_reasons": "|".join(map(str, reasons)), "mode": value.get("mode"),
                "road_connectivity": value.get("road_connectivity"),
                "safe_road_coverage": value.get("safe_road_coverage"),
                "left_boundary_points": value.get("left_boundary_points"),
                "right_boundary_points": value.get("right_boundary_points"),
                "gate_results": json.dumps(value.get("gate_results", {}),
                                             separators=(",", ":")),
            }
        except (TypeError, ValueError):
            pass

    def _append_jsonl(self, filename, text):
        with open(os.path.join(self.output_dir, filename),
                  "a", encoding="utf-8") as stream:
            stream.write(str(text)+"\n")

    @staticmethod
    def _rate(times):
        return 0.0 if len(times) < 2 else (len(times)-1)/max(1.0e-9, times[-1]-times[0])

    def _finish_when_due(self):
        if time.monotonic()-self.started >= self.duration:
            self._write_summary(); rclpy.shutdown()

    def _write_summary(self):
        if getattr(self, "_written", False): return
        self._written = True
        if self.writer is not None: self.writer.release(); self.writer = None
        overlay_times = self.arrivals.get("overlay", [])
        if len(overlay_times) >= 2 and os.path.isfile(self.video_path):
            encoded_duration = self.overlay_count/max(1.0e-9, self.record_fps)
            measured_duration = overlay_times[-1]-overlay_times[0]
            scale = measured_duration/max(1.0e-9, encoded_duration)
            temporary = self.video_path+".retimed.mp4"
            try:
                subprocess.run([
                    "ffmpeg", "-y", "-v", "error", "-itsscale", str(scale),
                    "-i", self.video_path, "-c", "copy", temporary],
                    check=True)
                os.replace(temporary, self.video_path)
            except (OSError, subprocess.SubprocessError) as error:
                self.get_logger().error(f"overlay timestamp retiming failed: {error}")
        trace_fields = [
            "frame", "source_stamp_ns", "state_stamp_ns", "planner_state",
            "path_point_count", "target_x_m", "target_y_m",
            "lateral_error_m", "heading_error_deg", "curvature_per_m",
            "target_curvature_per_m", "atan_wheelbase_curvature_deg",
            "raw_steering_deg", "required_steering_deg",
            "filtered_steering_deg", "published_camera_wheel",
        ]
        with open(os.path.join(self.output_dir, "steering_trace.csv"), "w",
                  newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=trace_fields)
            writer.writeheader()
            writer.writerows(sorted(self.controller_trace_by_source.values(),
                                    key=lambda item: item["source_stamp_ns"]))
        gate_fields = [
            "frame", "source_stamp", "road_pixels", "near_field_coverage",
            "w_line_pixels", "y_line_pixels", "corridor_width_m",
            "path_point_count", "path_length_m", "curvature_per_m",
            "required_steering_deg", "state", "primary_reason", "all_reasons",
            "mode", "road_connectivity", "safe_road_coverage",
            "left_boundary_points", "right_boundary_points", "gate_results"]
        with open(os.path.join(self.output_dir, "planner_gate_frames.csv"), "w",
                  newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=gate_fields)
            writer.writeheader()
            writer.writerows(sorted(self.gate_rows_by_source.values(),
                                    key=lambda item: item["source_stamp"]))
        summary = {
            "mode": self.mode,
            "measured_duration_sec": time.monotonic()-self.started,
            "unique_frames": {key: len(value) for key, value in self.unique.items()},
            "observed_fps": {key: self._rate(value) for key, value in self.arrivals.items()},
            "state_counts": dict(self.states),
            "empty_path_messages": self.empty_paths,
            "overlay_messages_recorded": self.overlay_count,
            "overlay_recording_fps": self._rate(overlay_times),
            "last_diagnostics": self.last_metrics,
            "last_upstream_diagnostics": self.upstream_metrics,
            "camera_drive": {
                "samples": len(self.drive_values),
                "min": min(self.drive_values, default=0.0),
                "max": max(self.drive_values, default=0.0),
                "counts": {str(value): self.drive_values.count(value)
                           for value in sorted(set(self.drive_values))},
                "invalid_nonzero_samples": self.causal_invalid_drive_nonzero,
                "arrival_order_invalid_nonzero_samples":
                    self.arrival_order_invalid_drive_nonzero,
            },
            "camera_wheel": {
                "samples": len(self.wheel_values),
                "min": min(self.wheel_values, default=0),
                "max": max(self.wheel_values, default=0),
                "nonzero_samples": sum(value != 0 for value in self.wheel_values),
                "invalid_nonzero_samples": self.causal_invalid_wheel_nonzero,
                "arrival_order_invalid_nonzero_samples":
                    self.arrival_order_invalid_wheel_nonzero,
            },
        }
        with open(os.path.join(self.output_dir, "measurement_summary.json"),
                  "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)

    def destroy_node(self):
        self._write_summary(); return super().destroy_node()


def main():
    rclpy.init(); node = ValidationRecorder()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
