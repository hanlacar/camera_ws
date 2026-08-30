# Camera path follower

`camera_path_following.launch.py` is the mission-free production chain:

`D456 -> TensorRT -> semantic -> image path -> metric path -> Pure Pursuit -> /cmd_drive + /cmd_wheel`

Command contracts:

- `/cmd_drive`: `std_msgs/msg/Float32`, `2.0` only while every safety gate is valid; otherwise `0.0`.
- `/cmd_wheel`: `std_msgs/msg/Int32`, integer steering degrees in `[-27, 27]`.
- `/control/camera_path_follower_diagnostics`: separate path-input and control-output rates plus safety reason and duplicate/stale/backlog counters.

The controller consumes only `/camera/path` and the exact-stamp
`/camera/metric_path_status`. It has no stop-line, traffic-light, sign, or
mission subscriptions. Keep mission selection in a separate module.

The checked-in camera mount remains deliberately unconfigured. Until real
vehicle mounting values are measured in `camera_mount.yaml` and D456 IMU
access is restored, calibration remains invalid and the controller publishes
the safe stop command. Do not replace those values with guessed geometry.

Launch:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch race_control camera_path_following.launch.py
```
