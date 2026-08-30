"""Convert the camera stack's discrete command contract into a cmd_vel Twist.

The camera controllers publish:
  /camera_drive  (Float32)  STOP=0.0, SLOW=1.0, CRUISE=2.0, FAST=3.0
  /camera_wheel  (Int32)    steering angle in degrees, clamped to +/-27

The Gazebo Ackermann plugin consumes geometry_msgs/Twist on /cmd_vel:
  linear.x   forward speed (m/s)
  angular.z  yaw rate (rad/s)  -- the plugin turns this into a steering angle

We map the discrete drive stage to a speed, and the wheel angle to a yaw rate
using the bicycle model: yaw_rate = v / wheelbase * tan(steer). This keeps the
commanded path curvature consistent with the intended steering angle at the
chosen speed.

All functions are pure and framework independent so they unit test off-sim.
"""
import math

STOP, SLOW, CRUISE, FAST = 0.0, 1.0, 2.0, 3.0


def drive_stage_to_speed(drive, slow_mps, cruise_mps, fast_mps):
    """Map a discrete drive stage to a forward speed in m/s.

    STOP -> 0. Unknown/non-finite -> 0 (fail safe, never fabricate motion).
    FAST is supported for completeness but the current controllers never emit
    it; it maps to fast_mps if it ever appears.
    """
    if not math.isfinite(drive):
        return 0.0
    table = {STOP: 0.0, SLOW: slow_mps, CRUISE: cruise_mps, FAST: fast_mps}
    return float(table.get(drive, 0.0))


def wheel_deg_to_yaw_rate(wheel_deg, speed_mps, wheelbase_m, max_wheel_deg=27.0):
    """Bicycle-model yaw rate (rad/s) for a steering angle at a given speed.

    yaw_rate = v / L * tan(delta). At zero speed the yaw rate is zero (a
    stationary Ackermann vehicle cannot rotate in place), which is physically
    correct for this plugin. The wheel angle is clamped to +/-max_wheel_deg.
    Sign convention: positive wheel_deg (left) -> positive yaw (CCW), matching
    ROS REP-103 and the Ackermann plugin.
    """
    if not all(math.isfinite(v) for v in (wheel_deg, speed_mps, wheelbase_m)):
        return 0.0
    if wheelbase_m <= 0.0:
        return 0.0
    delta = max(-max_wheel_deg, min(max_wheel_deg, wheel_deg))
    return float(speed_mps / wheelbase_m * math.tan(math.radians(delta)))


def command_to_twist(drive, wheel_deg, wheelbase_m,
                     slow_mps=0.4, cruise_mps=0.9, fast_mps=1.4,
                     max_wheel_deg=27.0):
    """Full conversion -> (linear_x, angular_z). Fail-safe to (0, 0)."""
    try:
        speed = drive_stage_to_speed(drive, slow_mps, cruise_mps, fast_mps)
        yaw = wheel_deg_to_yaw_rate(wheel_deg, speed, wheelbase_m, max_wheel_deg)
        if not (math.isfinite(speed) and math.isfinite(yaw)):
            return 0.0, 0.0
        return float(speed), float(yaw)
    except Exception:
        return 0.0, 0.0
