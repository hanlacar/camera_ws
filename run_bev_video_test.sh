#!/usr/bin/env bash
# Direct-BEV-only MP4 validation. Usage: ./run_bev_video_test.sh VIDEO [OUTPUT_DIR]
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIDEO="${1:-}"; OUTPUT_DIR="${2:-$WS/validation/bev}"
DURATION_SEC="${DURATION_SEC:-25}"; WARMUP_SEC="${WARMUP_SEC:-8}"
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1
if [[ -z "$VIDEO" || ! -f "$VIDEO" ]]; then echo "Usage: $0 VIDEO [OUTPUT_DIR]" >&2; exit 2; fi
VIDEO="$(realpath "$VIDEO")"
mkdir -p "$OUTPUT_DIR"
export ROS_LOG_DIR="$OUTPUT_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
existing="$(ros2 node list --no-daemon 2>/dev/null || true)"
if grep -Eq 'camera_image_path_node|adaptive_non_bev|camera_metric_path_node|camera_pixel_controller' <<<"$existing"; then
  echo "ERROR: a Non-BEV/legacy planner or pixel controller is already running." >&2; exit 3
fi
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]:-}"; do [[ -n "${pid:-}" ]] && kill "$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]:-}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
PT_MODEL="$WS/src/camera_yolo_inference/models/hanla_yolo11n_seg_best.pt"
PLANNER_VARIANT="${PLANNER_VARIANT:-production}"
LINE_TRACK_MODE="${LINE_TRACK_MODE:-none}"
# Validation must exercise the newly installed checkpoint itself. Existing
# TensorRT/ONNX artifacts were exported from older weights and are never used.
BACKEND=pytorch; MODEL="$PT_MODEL"; DEVICE=cpu; REQUIRE_CUDA=false
echo "MODEL VALIDATION: forcing PyTorch checkpoint $MODEL" | tee "$OUTPUT_DIR/backend_warning.log"
MODEL_SHA256="$(sha256sum "$MODEL" | cut -d' ' -f1)"
VIDEO_SHA256="$(sha256sum "$VIDEO" | cut -d' ' -f1)"
{
  echo "video_path=$VIDEO"
  echo "video_sha256=$VIDEO_SHA256"
} | tee "$OUTPUT_DIR/video_identity.log"
SOURCE_FPS="$(ffprobe -v error -select_streams v:0 -show_entries stream=avg_frame_rate -of csv=p=0 "$VIDEO" | awk -F/ '{printf "%.6f", $1/$2}')"
PUBLISH_RATE_HZ="${PUBLISH_RATE_HZ:-$SOURCE_FPS}"
PUBLISH_RATE_HZ="$(awk -v value="$PUBLISH_RATE_HZ" 'BEGIN {printf "%.6f", value}')"
DURATION_VALUE="$(awk -v value="$DURATION_SEC" 'BEGIN {printf "%.3f", value}')"
ros2 launch camera_yolo_inference yolo_inference.launch.py backend:="$BACKEND" segmentation_model_path:="$MODEL" expected_model_sha256:="$MODEL_SHA256" device:="$DEVICE" require_cuda:="$REQUIRE_CUDA" line_track_mode:="$LINE_TRACK_MODE" >"$OUTPUT_DIR/yolo.log" 2>&1 & PIDS+=("$!")
sleep 3
ros2 run camera_navigation direct_bev_planner_node --ros-args --params-file "$WS/src/camera_bringup/config/camera_mount.yaml" --params-file "$WS/src/camera_navigation/config/bev_path.yaml" -p planner_variant:="$PLANNER_VARIANT" >"$OUTPUT_DIR/planner.log" 2>&1 & PIDS+=("$!")
ros2 run camera_navigation direct_bev_controller_node --ros-args --params-file "$WS/src/camera_navigation/config/bev_controller.yaml" >"$OUTPUT_DIR/controller.log" 2>&1 & PIDS+=("$!")
ros2 run camera_navigation bev_wheel_selector_node --ros-args -p active_planner:=bev >"$OUTPUT_DIR/selector.log" 2>&1 & PIDS+=("$!")
ros2 run camera_navigation direct_bev_drive_node >"$OUTPUT_DIR/drive.log" 2>&1 & PIDS+=("$!")
sleep 1
python3 "$WS/bev_test_pub.py" --ros-args -p video_path:="$VIDEO" -p rate_hz:="$PUBLISH_RATE_HZ" -p loop:=true >"$OUTPUT_DIR/publisher.log" 2>&1 & PUBLISHER_PID="$!"; PIDS+=("$PUBLISHER_PID")
ps -p "$PUBLISHER_PID" -o args= | tee "$OUTPUT_DIR/publisher_process_args.log"
sleep "$WARMUP_SEC"
python3 "$WS/video_test_recorder.py" --ros-args -p mode:=bev -p output_dir:="$OUTPUT_DIR" -p duration_sec:="$DURATION_VALUE" -p record_fps:="$PUBLISH_RATE_HZ" >"$OUTPUT_DIR/recorder.log" 2>&1 & RECORDER_PID="$!"; PIDS+=("$RECORDER_PID")
for topic in /camera/image_raw /perception/semantic_path_frame /camera/bev/path /camera/bev/overlay_image /camera/bev/overlay /camera_drive /camera_wheel; do
  timeout "${DURATION_SEC}s" ros2 topic hz "$topic" --window 100 >"$OUTPUT_DIR/hz_${topic//\//_}.log" 2>&1 & PIDS+=("$!")
done
sleep 2
ros2 topic info -v /camera/bev/overlay_image >"$OUTPUT_DIR/topic_info_overlay_image.log" 2>&1 || true
ros2 topic info -v /camera_drive >"$OUTPUT_DIR/topic_info_camera_drive.log" 2>&1 || true
ros2 topic info -v /camera_wheel >"$OUTPUT_DIR/topic_info_camera_wheel.log" 2>&1 || true
ros2 topic info -v /camera/bev/wheel >"$OUTPUT_DIR/topic_info_bev_wheel.log" 2>&1 || true
ros2 node list >"$OUTPUT_DIR/node_list.log" 2>&1 || true
ros2 node info /bev_wheel_selector_node >"$OUTPUT_DIR/node_info_wheel_publisher.log" 2>&1 || true
ros2 param dump /direct_bev_controller_node >"$OUTPUT_DIR/controller_parameters.yaml" 2>&1 || true
ros2 param dump /direct_bev_drive_node >"$OUTPUT_DIR/drive_parameters.yaml" 2>&1 || true
timeout 5 ros2 topic echo /camera/bev/state --once >"$OUTPUT_DIR/state_once.log" 2>&1 || true
timeout 5 ros2 topic echo /camera/bev/diagnostics --once >"$OUTPUT_DIR/diagnostics_once.log" 2>&1 || true
timeout 5 ros2 topic echo /camera_drive --once >"$OUTPUT_DIR/camera_drive_once.log" 2>&1 || true
timeout 5 ros2 topic echo /camera_wheel --once >"$OUTPUT_DIR/camera_wheel_once.log" 2>&1 || true
timeout 5 ros2 topic echo /camera_drive >"$OUTPUT_DIR/camera_drive_values.log" 2>&1 || true
timeout 5 ros2 topic echo /camera_wheel >"$OUTPUT_DIR/camera_wheel_values.log" 2>&1 || true
wait "$RECORDER_PID"
echo "BEV validation complete: $OUTPUT_DIR"
