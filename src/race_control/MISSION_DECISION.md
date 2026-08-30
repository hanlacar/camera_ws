# Mission decision layer

The mission layer is an advisory longitudinal-permission module. It does not
own `/cmd_drive` or `/cmd_wheel`, does not calculate steering, and does not
subscribe to the geometric path controller.

Current stop-line inputs:

- `/perception/stop_detected` (`std_msgs/msg/Bool`)
- `/camera/perception_valid` (`std_msgs/msg/Bool`)
- `/mission/stop_line_release` (`std_msgs/msg/Bool`), explicit one-shot release

Outputs:

- `/mission/state` (`String`): `RUN`, `STOP`, or `WAIT`
- `/mission/speed_permission` (`Bool`)
- `/mission/speed_target` (`Float32`): `2.0` in `RUN`, otherwise `0.0`
- `/mission/decision` (`String` JSON): reason, counters, freshness, rates

Three positive frames confirm a stop line. The stop latches, observes the
minimum stop interval, then remains in `WAIT` until a release request and five
clear frames are both present. Missing or invalid perception fails closed to
`WAIT`. A disappearing detection never automatically resumes driving.

This output is intentionally not connected to the current controller yet.
Future traffic-light and sign modules should contribute longitudinal
constraints to this layer; they must not be merged into Pure Pursuit.
