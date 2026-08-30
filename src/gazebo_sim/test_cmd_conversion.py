import math, sys
sys.path.insert(0, "gazebo_sim")
from gazebo_sim.cmd_conversion import (
    STOP, SLOW, CRUISE, FAST,
    drive_stage_to_speed, wheel_deg_to_yaw_rate, command_to_twist)

def approx(a, b, t=1e-6): return abs(a-b) <= t
WB = 0.77

# --- drive stage -> speed ---
assert drive_stage_to_speed(STOP, .4, .9, 1.4) == 0.0
assert drive_stage_to_speed(SLOW, .4, .9, 1.4) == 0.4
assert drive_stage_to_speed(CRUISE, .4, .9, 1.4) == 0.9
assert drive_stage_to_speed(FAST, .4, .9, 1.4) == 1.4
assert drive_stage_to_speed(float('nan'), .4, .9, 1.4) == 0.0
assert drive_stage_to_speed(7.0, .4, .9, 1.4) == 0.0   # unknown -> 0

# --- wheel -> yaw rate ---
# straight -> 0
assert approx(wheel_deg_to_yaw_rate(0, 0.9, WB), 0.0)
# zero speed -> zero yaw even with steering
assert approx(wheel_deg_to_yaw_rate(20, 0.0, WB), 0.0)
# left (positive) -> positive yaw
assert wheel_deg_to_yaw_rate(20, 0.9, WB) > 0
# right (negative) -> negative yaw
assert wheel_deg_to_yaw_rate(-20, 0.9, WB) < 0
# value check: v/L*tan(delta)
expect = 0.9/WB*math.tan(math.radians(20))
assert approx(wheel_deg_to_yaw_rate(20, 0.9, WB), expect)
# clamp beyond max
assert approx(wheel_deg_to_yaw_rate(50, 0.9, WB),
              0.9/WB*math.tan(math.radians(27)))
# bad wheelbase -> 0
assert wheel_deg_to_yaw_rate(20, 0.9, 0.0) == 0.0

# --- full conversion ---
lx, az = command_to_twist(CRUISE, 0, WB)
assert approx(lx, 0.9) and approx(az, 0.0)
lx, az = command_to_twist(SLOW, 27, WB)
assert approx(lx, 0.4) and az > 0
lx, az = command_to_twist(STOP, 27, WB)
assert approx(lx, 0.0) and approx(az, 0.0)   # stopped -> no motion at all
# nonfinite -> (0,0)
assert command_to_twist(float('nan'), 10, WB) == (0.0, 0.0)

print("CMD CONVERSION TESTS PASSED")
