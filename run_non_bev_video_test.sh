#!/usr/bin/env bash
# Non-BEV-only MP4 validation. Usage: ./run_non_bev_video_test.sh VIDEO [OUTPUT_DIR]
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIDEO="${1:-}"; OUTPUT_DIR="${2:-$WS/validation/non_bev}"
DURATION_SEC="${DURATION_SEC:-25}"; WARMUP_SEC="${WARMUP_SEC:-8}"
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [[ -z "$VIDEO" || ! -f "$VIDEO" ]]; then echo "Usage: $0 VIDEO [OUTPUT_DIR]" >&2; exit 2; fi
mkdir -p "$OUTPUT_DIR"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
existing="$(ros2 node list --no-daemon 2>/dev/null || true)"
if grep -Eq 'direct_bev|camera_metric_path_node|direct_bev_controller|bev_wheel_selector' <<<"$existing"; then
  echo "ERROR: a BEV/legacy metric planner or BEV controller is already running." >&2; exit 3
fi
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]:-}"; do [[ -n "${pid:-}" ]] && kill "$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]:-}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
PT_MODEL="$WS/src/camera_yolo_inference/models/hanla_yolo11n_seg_best.pt"
# Validation must exercise the newly installed checkpoint itself. Existing
# TensorRT/ONNX artifacts were exported from older weights and are never used.
BACKEND=pytorch; MODEL="$PT_MODEL"; DEVICE=cpu; REQUIRE_CUDA=false
echo "MODEL VALIDATION: forcing PyTorch checkpoint $MODEL" | tee "$OUTPUT_DIR/backend_warning.log"
MODEL_SHA256="$(sha256sum "$MODEL" | cut -d' ' -f1)"
SOURCE_FPS="$(ffprobe -v error -select_streams v:0 -show_entries stream=avg_frame_rate -of csv=p=0 "$VIDEO" | awk -F/ '{printf "%.6f", $1/$2}')"
DURATION_VALUE="$(awk -v value="$DURATION_SEC" 'BEGIN {printf "%.3f", value}')"
ros2 launch camera_yolo_inference yolo_inference.launch.py backend:="$BACKEND" segmentation_model_path:="$MODEL" expected_model_sha256:="$MODEL_SHA256" device:="$DEVICE" require_cuda:="$REQUIRE_CUDA" >"$OUTPUT_DIR/yolo.log" 2>&1 & PIDS+=("$!")
sleep 3
ros2 launch camera_navigation image_path.launch.py require_control_mode:=false >"$OUTPUT_DIR/planner.log" 2>&1 & PIDS+=("$!")
sleep 1
python3 "$WS/bev_test_pub.py" --ros-args -p video_path:="$VIDEO" -p rate_hz:="$SOURCE_FPS" -p loop:=true >"$OUTPUT_DIR/publisher.log" 2>&1 & PIDS+=("$!")
sleep "$WARMUP_SEC"
python3 "$WS/video_test_recorder.py" --ros-args -p mode:=non_bev -p output_dir:="$OUTPUT_DIR" -p duration_sec:="$DURATION_VALUE" -p record_fps:="$SOURCE_FPS" >"$OUTPUT_DIR/recorder.log" 2>&1 & RECORDER_PID="$!"; PIDS+=("$RECORDER_PID")
for topic in /camera/image_raw /perception/semantic_path_frame /camera/path_overlay_image /camera/path_debug_image /camera/image_path_typed; do
  timeout "${DURATION_SEC}s" ros2 topic hz "$topic" --window 100 >"$OUTPUT_DIR/hz_${topic//\//_}.log" 2>&1 & PIDS+=("$!")
done
sleep 2
ros2 topic info -v /camera/path_overlay_image >"$OUTPUT_DIR/topic_info_overlay_image.log" 2>&1 || true
timeout 5 ros2 topic echo /camera/image_path_state --once >"$OUTPUT_DIR/state_once.log" 2>&1 || true
timeout 5 ros2 topic echo /camera/path_realtime_fps --once >"$OUTPUT_DIR/realtime_fps_once.log" 2>&1 || true
wait "$RECORDER_PID"
echo "Non-BEV validation complete: $OUTPUT_DIR"
