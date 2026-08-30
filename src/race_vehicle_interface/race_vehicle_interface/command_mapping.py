import math


CAMERA_DRIVE_VALUES = (0.0, 1.0, 2.0, 3.0)


def camera_drive_to_stage(value):
    """Adapt the discrete Float32 camera command to the proven ROS stage.

    The boolean result lets the bridge distinguish a legitimate STOP from an
    invalid value that must also disarm command transmission.
    """
    if isinstance(value, bool):
        return 0, False
    try:
        command = float(value)
    except (TypeError, ValueError):
        return 0, False
    if not math.isfinite(command) or command not in CAMERA_DRIVE_VALUES:
        return 0, False
    return int(command), True


def camera_wheel_to_steer(value, maximum_steering_deg=27):
    """Adapt the Int32 camera wheel contract to steering degrees."""
    if isinstance(value, bool) or not isinstance(value, int):
        return 0.0, False
    maximum = int(maximum_steering_deg)
    if maximum <= 0 or abs(value) > maximum:
        return 0.0, False
    return float(value), True


def clamp(value, lower, upper):
    if not math.isfinite(value):
        return 0.0
    return max(lower, min(upper, value))


def speed_to_stage(speed_mps, stage_per_mps, max_abs_stage, stage_sign=1):
    """Convert target speed to an integer stage with explicit calibration."""
    values = (speed_mps, stage_per_mps, stage_sign)
    if not all(math.isfinite(float(value)) for value in values):
        return 0
    if stage_per_mps <= 0.0 or max_abs_stage <= 0:
        return 0
    raw = round(float(speed_mps) * float(stage_per_mps) * float(stage_sign))
    return int(max(-int(max_abs_stage), min(int(max_abs_stage), raw)))


def steering_command(target_deg, maximum_deg, steering_sign=1.0):
    if maximum_deg <= 0.0 or not math.isfinite(float(steering_sign)):
        return 0.0
    requested = float(target_deg) * float(steering_sign)
    return float(clamp(requested, -float(maximum_deg), float(maximum_deg)))
