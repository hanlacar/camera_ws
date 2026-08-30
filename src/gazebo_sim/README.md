# gazebo_sim

Gazebo (gz-sim8) test harness for the turtle_car camera driving stack on the
S-curve world. Runs the **real** YOLO + camera_navigation stack against a
simulated camera, converts the stack's discrete commands to `/cmd_vel`, and
drives the existing Ackermann vehicle.

## What this package adds
- `urdf/turtle_car_with_camera.urdf.xacro` — the existing vehicle + a camera
  and IMU sized to the real vehicle (rear-axle→camera 0.31 m, height 0.80 m).
  Because this sim's `base_link` is at the vehicle center, the camera's
  `base_link` x is `-wheel_base/2 + 0.31 = -0.075 m`.
- `gazebo_sim/cmd_vel_adapter_node.py` — `/camera_drive` + `/camera_wheel`
  (discrete) → `/cmd_vel` (Twist), with a stop-on-stale fail-safe.
- `gazebo_sim/imu_splitter_node.py` — one Gazebo `/camera/imu` → the
  `accel/sample` + `gyro/sample` topics the BEV metric node expects.
- Launch files for the full sim with either controller.

## Build
```
cd ~/camera_ws
colcon build --packages-select gazebo_sim
source install/setup.bash
```
Requires: `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image` (Gazebo Harmonic /
gz-sim8), plus the existing `camera_navigation`, `camera_yolo_inference`,
`race_perception`, `race_interfaces` packages.

## Run — non-BEV (pixel) pipeline (try this first)
No camera extrinsics / IMU needed. Best while camera pitch is still changing.
```
ros2 launch gazebo_sim pixel_control.launch.py
```

## Run — BEV pipeline
Keep the car still for the first ~1–2 s so the IMU attitude lock succeeds.
Uses `config/camera_mount_sim.yaml` (NOT the real camera_mount.yaml).
```
ros2 launch gazebo_sim bev_control.launch.py
```

## The YOLO-recognition risk (read this)
We deliberately run the real YOLO model. If it does not recognize the rendered
road/lanes, `/perception/semantic_path_frame` stays empty, no path is produced,
and the controllers fail-safe to STOP (the car won't move). To check whether
YOLO is seeing the road:
```
ros2 topic echo /camera/navigation_mask_available   # expect: true
ros2 run rqt_image_view rqt_image_view /camera/perception_overlay_image
ros2 topic echo /camera/inference_status
```
If YOLO does not recognize the sim road, options are: fine-tune / swap the
model, or add a sim-only classical lane detector that publishes the same
`/perception/semantic_path_frame` contract (not included here).

## Watch the pipeline
```
ros2 topic echo /camera/image_path_typed        # pixel path (should populate)
ros2 topic echo /camera/path                    # metric path (BEV only)
ros2 topic echo /camera_drive                   # 2.0 straight, 1.0 in curves
ros2 topic echo /camera_wheel                   # steering deg
ros2 topic echo /cmd_vel                         # adapter output
ros2 topic echo /camera/pixel_controller_diagnostics   # pixel pipeline
ros2 topic echo /camera/controller_diagnostics         # BEV pipeline
```

## Key geometry (must stay in sync)
| quantity | value | where |
|---|---|---|
| wheel_base | 0.77 m | turtle_car_urdf.xacro |
| wheel_radius | 0.18 m | turtle_car_urdf.xacro |
| camera x (base_link) | -0.075 m | camera_sensor.xacro, camera_mount_sim.yaml |
| camera height | 0.80 m | camera_sensor.xacro, camera_mount_sim.yaml |
| camera pitch | ~5° down | camera_sensor.xacro (+0.0873 rad) / mount_sim (-5°) |
| spawn pose | x=3.25, y=0, yaw=0 | s_curve_avoidance.sdf, sim_base.launch.py |
