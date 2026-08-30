# D456 카메라 중앙주행 워크스페이스

## 환경

- Ubuntu 24.04
- ROS 2 Jazzy
- Intel RealSense D456
- NVIDIA CUDA/TensorRT
- `ROS_DOMAIN_ID=12`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

이 워크스페이스의 실차 기준은 BEV가 아닌 pixel controller입니다. 정상
경로에서 `|camera_wheel| < 5`이면 `camera_drive=2.00`, 5도 이상이면
`camera_drive=1.00`입니다. Wheel 범위는 -27~+27도이며 양수는 오른쪽,
음수는 왼쪽입니다. Invalid, stale, timeout 또는 비정상 입력은 drive 0.00과
wheel 0으로 닫힙니다.

정지선 종방향 제어는 D456의 color-aligned depth와 기존 YOLO `stop_line`
mask를 사용합니다. 판단 거리는 카메라가 아니라 앞범퍼 기준입니다.

- 정지선 거리 `> 2.0m`: 기존 steering 기반 주행
- `0.7m < 거리 <= 2.0m`: `camera_drive=1.00`, `camera_stop=False`
- 거리 `<= 0.7m`: `camera_drive=0.00`, `camera_stop=True`

정지 중에도 유효한 중앙경로 조향은 유지됩니다. Path fail-safe가 발생하면 더
높은 우선순위로 drive 0.00과 wheel 0을 출력합니다. Stop은 연속 프레임 확인 후
latch되므로 정지선 mask나 depth가 한 프레임 사라져도 자동 출발하지 않습니다.
실차 테스트 전에 `camera_to_front_bumper_m`를 카메라 광학 중심부터 앞범퍼까지
실측한 값으로 설정해야 합니다.

신호등 상태는 기존 TensorRT segmentation의 `R_light`, `Y_light`, `G_light`
결과에서 자동 판단되어 `/camera_traffic_light` (`std_msgs/msg/String`)로
발행됩니다.

- `R`: 적색
- `Y`: 황색
- `G`: 녹색
- `UNKNOWN`: 미확정, 검출 소실 또는 색상 충돌

확정에 필요한 연속 프레임 수, 검출 소실 timeout, 최소 confidence는 각각
`traffic_light_confirmation_frames`, `traffic_light_lost_timeout_sec`,
`traffic_light_minimum_confidence` parameter로 조정할 수 있습니다. 이 출력은
판단 전용이며 drive, wheel, stop 또는 정지 latch를 변경하지 않습니다.

## 공통 인지 정제 및 BEV overlay

YOLO class/confidence와 `/perception/masks/*` raw semantic mask는 변경하지
않습니다. `camera_yolo_inference_node`가 RGB와 각 line instance ROI를 함께
분석해 HSV/Lab·주변 Road 대비, 정규화 형상, IoU/중심점 temporal track 및
hysteresis를 적용하고, 하나의 refined `SemanticPathFrame`을 Direct BEV와
Non-BEV에 전달합니다. 따라서 두 planner의 색상/형상 재판정은 동일합니다.

주요 추가 출력은 다음과 같습니다.

- `/perception/refined/{road,white_line,yellow_line,unknown_line}`
- `/perception/refined/{stop_line_candidate,crosswalk_candidate,words}`
- `/perception/refined/{road_marking_unknown,restored_markings}`
- `/camera/perception_refinement_diagnostics` (raw/refined pixel 수, 색상 근거,
  track ID, stop/crosswalk confidence, Road 복원 근거)
- `/camera/bev/overlay_image` (실제 RGB의 calibrated BEV warp, raw/refined/safe
  Road, 흰/노란선, 복원 영역, path, 차량 기준점, mode/state/confidence/steering)

정제 임계값은
`src/camera_yolo_inference/config/yolo_inference.yaml`의 `refinement.*`, BEV
overlay는 `src/camera_navigation/config/bev_path.yaml`의
`bev_overlay_*`에서 설정합니다. Overlay는 subscriber가 있을 때만 exact-stamp
RGB를 latest-only worker에서 warp/render하며, 비구독 시 이 경로는 실행되지
않습니다. `/camera/realtime_fps`, `/camera/path_realtime_fps`,
`/camera/bev/diagnostics`에서 camera/inference/refinement/각 planner/overlay의
고유 timestamp FPS, 교체 frame 수 및 p50/p95 latency를 확인합니다. 동일
frame을 새 입력처럼 재발행하지 않습니다.

## 1. 새 노트북 설치

```bash
git clone https://github.com/hanlacar/camera_ws.git
cd urrc_hanla
```

ROS 2 Jazzy를 설치한 뒤 필요한 ROS 패키지와 의존성을 설치합니다.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y \
  python3-rosdep python3-colcon-common-extensions \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-realsense2-camera \
  ros-jazzy-rqt-image-view
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

YOLO와 TensorRT Python 환경은 사용하는 NVIDIA 드라이버/CUDA 버전에 맞게
설치합니다. TensorRT engine은 GPU와 런타임에 종속되므로 새 노트북에서
명시적으로 생성합니다.

```bash
python3 src/camera_yolo_inference/tools/export_tensorrt_engine.py \
  --model src/camera_yolo_inference/models/hanla_yolo11n_seg_best.pt \
  --output src/camera_yolo_inference/models/hanla_yolo11n_seg_best.engine
```

## 2. D456 연결 및 장치 선택

D456을 USB 3.x 포트에 연결하고 인식 상태를 확인합니다.

```bash
rs-enumerate-devices -s
rs-enumerate-devices | grep -E 'Name|Serial Number|Firmware Version|Usb Type Descriptor'
lsusb -t
```

RealSense가 한 대이면 launch가 자동 탐색하므로 serial을 입력하지 않습니다.
여러 대가 연결된 경우에만 위 출력의 D456 serial을 외부에서 지정합니다.

```bash
export D456_SERIAL=YOUR_D456_SERIAL
```

RealSense 접근 권한 오류가 있으면 librealsense udev rules를 설치하고 카메라를
다시 연결한 후 일반 사용자로 재확인합니다. `/dev/video*` 번호나 USB 물리
포트는 장치 식별자로 사용하지 않습니다.

## 3. 빌드

```bash
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install --packages-up-to \
  camera_bringup camera_yolo_inference camera_navigation
source install/setup.bash
```

아래 분리 실행은 각각 새 터미널에서 수행합니다. RealSense가 한 대인 기본
상황에는 `serial_no` 인자가 필요하지 않습니다.

## 4. 터미널 1 — 인지 실행

D456 RGB 영상을 YOLO segmentation semantic 결과로 변환합니다.

```bash
cd camera_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch camera_yolo_inference d456_yolo_perception.launch.py launch_rqt:=false
```

여러 RealSense 중 하나를 선택할 때만 마지막 명령에
`serial_no:="$D456_SERIAL"`을 추가합니다.

## 5. 터미널 2 — 중앙경로 생성

YOLO semantic 결과로 image pixel 중앙경로를 생성합니다. Standalone launch는
별도 mission/control mode 토픽을 요구하지 않으며 path safety gate는 유지합니다.

```bash
cd camera_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch camera_navigation image_path.launch.py
```

## 6. 터미널 3 — Pixel 제어 실행

유효한 중앙경로를 `/camera_drive`, `/camera_wheel` 출력으로 변환합니다.

```bash
cd camera_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch camera_navigation camera_pixel_controller.launch.py
```

## 7. 전체 파이프라인 한 번에 실행

분리 실행 중인 세 터미널을 먼저 종료한 뒤 D456→YOLO→중앙경로→pixel
controller 전체를 한 번에 실행할 수 있습니다. 이 launch는 RQT를 실행하지
않습니다.

```bash
cd camera_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch camera_navigation camera_pixel_control.launch.py
```

여러 RealSense 중 하나를 선택할 때만 마지막 명령에
`serial_no:="$D456_SERIAL"`을 추가합니다.

## 8. RQT 별도 실행

인지·경로·제어 launch와 다른 터미널에서 원본 기반 perception overlay와 최종
path overlay를 동시에 확인합니다.

```bash
cd camera_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch camera_bringup two_view_rqt.launch.py
```

Direct BEV와 Non-BEV를 동시에 비교 실행하고 새 RGB BEV overlay를 여는 명령은
다음과 같습니다.

```bash
cd camera_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch camera_navigation camera_dual_planner_compare.launch.py \
  active_planner:=none debug:=false
```

다른 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rqt_image_view /camera/bev/overlay_image
```

실시간 진단은 이미지 토픽을 CLI로 역직렬화하지 말고 다음 small JSON 토픽으로
확인합니다.

```bash
ros2 topic echo /camera/realtime_fps
ros2 topic echo /camera/path_realtime_fps
ros2 topic echo /camera/bev/diagnostics
```

## 9. 야외 정적 테스트 순서

차량 구동부를 연결하거나 움직이기 전에 정지 상태로 다음을 확인합니다.

1. **직선:** 경로가 차량 중심에서 시작해 도로 중앙으로 이어지고 wheel이 0
   근처인지, 프레임마다 좌우로 흔들리지 않는지 확인합니다.
2. **왼쪽 곡선:** 경로가 왼쪽 곡률을 따라가고 `camera_wheel`이 음수인지
   확인합니다.
3. **오른쪽 곡선:** 경로가 오른쪽 곡률을 따라가고 `camera_wheel`이 양수인지
   확인합니다.
4. **한쪽 차선:** 보이는 차선 자체를 주행 경로로 사용하지 않고 차량 중앙
   쪽으로 복원되며 해당 경계의 곡률을 유지하는지 확인합니다.
5. **햇빛·그림자:** 강한 햇빛, 그림자, 역광, 부분 음영에서 경로가 순간적으로
   큰 폭으로 점프하거나 조향 부호가 반복 반전되지 않는지 확인합니다.

경로가 도로 밖이나 차선 위에 생성되거나 직선에서 큰 조향이 반복되면 차량을
움직이지 말고 overlay와 터미널 로그를 보관합니다.

## 10. IMU 오르막 경사 판단

`imu_manager`가 유효한 IMU 데이터를 처리하면 다음 상태를 자동으로 발행합니다.

```text
/imu/slope

True  = 오르막 경사로
False = 평지 또는 내리막
```

오르막 진입·해제 pitch 기준은 각각 `uphill_enter_pitch_deg`,
`uphill_exit_pitch_deg` 파라미터로 조정할 수 있으며,
`uphill_confirmation_frames`로 연속 확인 프레임 수를 조정할 수 있습니다.

오르막 정지는 `/imu/slope`의 유효한 `False → True` 진입에서 한 번만
동작합니다.

```text
오르막 진입: /imu/slope False → True

→ 5초 정지
→ 5초 후 자동 재주행
→ 같은 오르막에서는 재정지하지 않음
→ 평지 복귀 후 다음 오르막에서 다시 1회 정지
```

정지 시간은 pixel controller의 `uphill_stop_duration_sec` 파라미터로
조정할 수 있으며 기본값은 다음과 같습니다.

```yaml
uphill_stop_duration_sec: 5.0
```

## 11. BEV metric reference path 준비

카메라 장착값은 뒷바퀴 중심축을 `base_link` 원점으로 하여 다음과 같이
적용합니다.

```text
height = 0.80m
rear axle 기준 x = +0.32m
y = 0.00m
pitch = -10°
yaw = 0°
roll = 0°
```

기존 ground-plane projection은 유효한 경우에만 `/camera/path`
(`nav_msgs/msg/Path`, `frame_id=base_link`, meter)를 발행합니다. 점은 차량에
가까운 곳부터 전방 순서이며 NaN/Inf, 역순, 중복, 과도한 점 간격,
self-intersection과 비정상적으로 짧은 경로를 검사합니다. 실제 투영 범위와
곡률은 `/camera/metric_path_status`에서 확인할 수 있습니다. 7~10m가 보이지
않는 경우 가짜 점을 외삽하지 않습니다.

현재 odom은 `camera_ws`에서 생성하지 않습니다. 속도·IMU 적분, visual
odometry 또는 production fake odom publisher도 포함하지 않습니다.

향후 외부 odom 소유자가 다음 계약을 제공하면:

```text
/odom
odom → base_link TF
```

camera reference adapter가 각 camera Path timestamp의 TF를 사용하여 다음
누적 경로를 생성합니다.

```text
/avoidance/route/reference_path
nav_msgs/msg/Path
frame_id = odom
```

TF가 없으면 adapter는 `waiting_for_tf` 상태로 기존 경로를 보존하고 새
reference를 발행하지 않습니다. TF가 나중에 나타나면 재시작 없이 동작합니다.
MCU의 유효 설정은 참고용 odom을 `/mcu/odom`으로 발행하며 `/odom`을
발행하지 않습니다. camera_ws는 MCU 거리·속도를 `/mcu/distance_m`과
`/mcu/speed_mps`에서 직접 구독하고 `/odom`으로 재계산하지 않습니다.

기존 D456·YOLO·pixel path가 실행 중일 때 metric path와 adapter를 함께
준비하려면 다음 launch를 사용합니다. 현재 실제 odom/TF가 없으면 adapter가
대기하는 것이 정상입니다.

```bash
ros2 launch camera_navigation camera_metric_reference.launch.py
```

## 12. 최종 출력 확인

마지막 별도 터미널에도 동일한 ROS 환경을 적용한 후 아래 두 출력만 확인합니다.

```bash
cd urrc_hanla
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

```bash
ros2 topic echo /camera_drive
```

```bash
ros2 topic echo /camera_wheel
```
# Independent MP4 overlay validation

ROS 2 Jazzy validation uses domain 12 and CycloneDDS. The two planners must be
run separately; the scripts reject a visible conflicting planner/controller.

```bash
./run_non_bev_video_test.sh ../20260827_070334_from_0530.mp4 validation/non_bev
./run_bev_video_test.sh ../20260827_070334_from_0530.mp4 validation/bev
```

`DURATION_SEC` defaults to 25 seconds after an 8-second warm-up. The scripts
select the newest `.engine` when both it and an NVIDIA driver are available.
They emit an explicit warning and use the `.pt` CPU backend otherwise. Each run
records topic rates, verbose overlay QoS, state/diagnostics, an overlay MP4, a
representative PNG, and `measurement_summary.json` under its output directory.
