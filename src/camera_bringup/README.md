# camera_bringup

Minimal ROS 2 Jazzy bring-up for one Intel RealSense D456. It starts the
installed `realsense2_camera_node` once, enables RGB plus the minimum aligned
depth stream required for stop-line ranging, and remaps those outputs to a
stable interface.
No relay, image conversion, RViz, rosbag, or composable container is started.

## Connection and USB check

Connect the D456 directly to a USB 3.x port with a USB 3-capable cable. The
default launch automatically selects the connected RealSense. Check
discovery, firmware, profiles, USB link information, and the device serial
with:

```bash
rs-enumerate-devices -s
rs-enumerate-devices | grep -E 'Name|Serial Number|Firmware Version|Usb Type Descriptor'
lsusb -t
```

The USB descriptor should report `3.x` and `lsusb -t` should show `5000M` or
faster. This launch file deliberately does not add a process to inspect USB.

## Build and run

```bash
cd camera_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select camera_bringup
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch camera_bringup d456_bringup.launch.py
```

The one-command `d456_production.launch.py` sets
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` for all of its child processes by
default. On this host, a 30-second D456 A/B test showed 59.64 FPS with
CycloneDDS versus 26.60 FPS through the default Fast DDS large-image path.
Use `rmw_implementation:=...` only for an explicit middleware A/B test.

With one D456 connected, no serial argument is needed. Select a serial only
when multiple RealSense devices are connected, or override the RGB profile:

```bash
ros2 launch camera_bringup d456_bringup.launch.py serial_no:=YOUR_D456_SERIAL
ros2 launch camera_bringup d456_bringup.launch.py \
  color_width:=640 color_height:=480 color_fps:=60
```

Only positive integer dimensions and FPS are accepted. The requested profile
must be supported by the connected camera.

## ROS interface and checks

The wrapper normally publishes below `/camera/camera`. Launch remaps the
control-path endpoints without copying image data:

| Wrapper endpoint | Public endpoint | Type |
|---|---|---|
| `/camera/camera/color/image_raw` | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera/color/camera_info` | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera/camera/aligned_depth_to_color/image_raw` | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` |

Inspect the streams with:

```bash
ros2 topic type /camera/image_raw
ros2 topic type /camera/camera_info
ros2 topic echo /camera/image_raw --once --field width
ros2 topic echo /camera/image_raw --once --field height
timeout 10s ros2 topic echo /camera/camera_info --once
ros2 topic info -v /camera/image_raw
ros2 topic info -v /camera/camera_info
```

Do not use `ros2 topic hz` as a production acceptance measurement for this
raw 0.92 MB image stream: its own subscription/deserialization can perturb the
path. Use the node's unique-stamp `/camera/realtime_fps` diagnostics or the
lightweight Stage 2 header-stamp probe instead.

The RealSense publisher uses its supported `color_qos: SENSOR_DATA` setting;
no publisher or subscriber is added by this package.

## Design

The installed wrapper 4.58.1 exposes `realsense2_camera_node` and `rs_launch.py`.
This package starts the executable directly because the installed include file
does not expose a remapping argument. Direct launch provides the same parameter
interface while making the two required remaps explicit and runs exactly one
camera node. The ROS parameter file uses the fully qualified `/camera/camera`
node key; launch supplies a serial only when explicitly requested and always
overrides the `WIDTHxHEIGHTxFPS` profile.

Color uses `RGB8`, automatic exposure (which also leaves gain automatic), and
640x480 at 60 Hz by default (the highest FPS the D456 RGB sensor supports at
640x480). Depth uses Z16 at 640x480x30 and the wrapper aligns it to color
coordinates. Synchronization and alignment are enabled only for this RGB/depth
pair. Infrared, RGBD, point cloud, and TF publication remain disabled. Gyro and
accel retain their existing production settings.

The installed Wrapper 4.58.1 publishes the raw IMU inputs as
`/camera/camera/gyro/sample` and `/camera/camera/accel/sample`, both with
`sensor_msgs/msg/Imu`. `camera_bringup` remains the only process that opens the
D456; IMU consumers must subscribe to these topics rather than starting another
RealSense node.

## Troubleshooting

If no image or CameraInfo appears:

1. Run `rs-enumerate-devices -s` and confirm the requested serial exists.
2. Run `lsusb -t` and confirm a 5000M-or-faster USB link; change the cable/port
   if it is 480M (USB 2.0).
3. Confirm `ros2 pkg prefix realsense2_camera` resolves under `/opt/ros/jazzy`.
4. Read the camera-node error on screen. A missing device times out after 10
   seconds; an unsupported RGB profile is reported by the wrapper.
5. Confirm the installed wrapper arguments with
   `ros2 launch realsense2_camera rs_launch.py --show-args` and compare names
   with `config/d456.yaml` after wrapper upgrades.
6. Confirm the installed parameter file exists with
   `ros2 pkg prefix camera_bringup`, then inspect
   `share/camera_bringup/config/d456.yaml` below that prefix.
7. Use `ros2 node list`, `ros2 topic list`, and the commands above to distinguish
   a missing image publisher from a missing CameraInfo publisher.

The launch shuts down when the RealSense process exits, so startup failures are
visible rather than leaving an apparently running bring-up process.

## Optional two-view RQT validation

Production never starts RQT unless `launch_rqt:=true` is explicitly supplied.
That option opens exactly two Image View processes: the canonical perception
overlay (`/camera/perception_overlay_image`) and the final path overlay
(`/camera/path_overlay_image`). The window manager can tile them left/right.
Both streams target 45 FPS by default, independently of the 60 FPS production
chain. Production inference/semantic/path rates remain separate metrics and
take priority if optional visualization cannot sustain 45 FPS safely.

The same two views can be started separately from production with:

```bash
ros2 launch camera_bringup two_view_rqt.launch.py
```

`/perception/detections_image` and `/camera/path_debug_image` remain detailed
development topics and are not opened by this launch.
