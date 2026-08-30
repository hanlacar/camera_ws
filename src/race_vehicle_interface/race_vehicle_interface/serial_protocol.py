import math
import re


LEGACY_STATUS_PATTERN = re.compile(
    r"\[상태\].*?A0=(-?\d+).*?편차=(-?\d+).*?누적=(-?\d+)(?:/-?\d+)?"
    r".*?RPM=([-+]?\d+(?:\.\d+)?)"
    r".*?오도=(-?\d+)"
)


def _parse_csv_status(line):
    """Parse fields confirmed by the current Arduino STATUS output source.

    The retained source defines the first eight fields as
    STATUS,state,fault,adc,target_adc,drive_pwm,rpm,encoder_count.  The
    deployed controller currently appends three numeric fields whose meaning
    is not present in that source, so they are validated but not interpreted.
    """
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("utf-8")
        except UnicodeDecodeError:
            return None
    fields = str(line).strip().split(",")
    if len(fields) not in (8, 11) or fields[0] != "STATUS":
        return None
    if not fields[1] or not fields[2]:
        return None
    try:
        steering_a0 = int(fields[3])
        target_a0 = int(fields[4])
        drive_pwm = int(fields[5])
        rpm = float(fields[6])
        encoder_count = int(fields[7])
        unknown_tail = tuple(float(value) for value in fields[8:])
    except ValueError:
        return None
    if not math.isfinite(rpm) or not all(
        math.isfinite(value) for value in unknown_tail
    ):
        return None
    if not -(2**63) <= encoder_count < 2**63:
        return None
    return {
        "state": fields[1],
        "fault": fields[2],
        "steering_a0": steering_a0,
        "target_a0": target_a0,
        "drive_pwm": drive_pwm,
        "rpm": rpm,
        "encoder_count": encoder_count,
    }


def parse_telemetry(line):
    """Parse T,<encoder_count>,<rpm>,<steering_position_ms>."""
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("utf-8")
        except UnicodeDecodeError:
            return None
    fields = str(line).strip().split(",")
    if len(fields) != 4 or fields[0] != "T":
        return None
    try:
        encoder_count = int(fields[1])
        rpm = float(fields[2])
        steering_angle = float(fields[3])
    except ValueError:
        return None
    if not math.isfinite(rpm) or not math.isfinite(steering_angle):
        return None
    if not -(2**63) <= encoder_count < 2**63:
        return None
    return encoder_count, rpm, steering_angle


def parse_legacy_status(line):
    """Parse supported on-demand S responses into the T-frame contract.

    Example: [상태] A0=359 편차=-4 누적=0/440 RPM=0.0 오도=0 PWM=0
    Returns (odometry_count, rpm, accumulated_steering_ms).

    CSV STATUS does not document accumulated steering time, so its third
    return value is None.  Callers must not fabricate a steering position.
    """
    csv_status = _parse_csv_status(line)
    if csv_status is not None:
        return csv_status["encoder_count"], csv_status["rpm"], None
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("utf-8")
        except UnicodeDecodeError:
            return None
    match = LEGACY_STATUS_PATTERN.search(str(line).strip())
    if match is None:
        return None
    steering_position_ms = float(match.group(3))
    rpm = float(match.group(4))
    encoder_count = int(match.group(5))
    if not math.isfinite(rpm) or not math.isfinite(steering_position_ms):
        return None
    return encoder_count, rpm, steering_position_ms


def parse_legacy_steering_a0(line):
    """Return the raw A0 steering sensor reading from a supported status."""
    csv_status = _parse_csv_status(line)
    if csv_status is not None:
        return csv_status["steering_a0"]
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("utf-8")
        except UnicodeDecodeError:
            return None
    match = LEGACY_STATUS_PATTERN.search(str(line).strip())
    return None if match is None else int(match.group(1))


def steering_position_to_degrees(
    steering_position_ms, maximum_position_ms, maximum_steering_deg
):
    """Convert v14 accumulated steering time to a clamped angle estimate."""
    steering_position_ms = float(steering_position_ms)
    maximum_position_ms = float(maximum_position_ms)
    maximum_steering_deg = float(maximum_steering_deg)
    if not all(
        math.isfinite(value)
        for value in (
            steering_position_ms,
            maximum_position_ms,
            maximum_steering_deg,
        )
    ):
        raise ValueError("steering conversion values must be finite")
    if maximum_position_ms <= 0.0 or maximum_steering_deg <= 0.0:
        raise ValueError("steering conversion maxima must be positive")
    estimated = steering_position_ms / maximum_position_ms * maximum_steering_deg
    return max(-maximum_steering_deg, min(maximum_steering_deg, estimated))


DRIVE_STAGE_TO_FIRMWARE = {
    0: 1,
    1: 2,
    2: 3,
    3: 4,
    -1: 6,
    -2: 7,
    -3: 8,
}


def encode_drive_command(stage, maximum_abs_stage):
    """Map signed ROS stages (-3..3) to the existing firmware command."""
    if isinstance(stage, bool) or not isinstance(stage, int):
        raise ValueError("stage must be an integer")
    maximum_abs_stage = int(maximum_abs_stage)
    if not 1 <= maximum_abs_stage <= 3:
        raise ValueError("maximum_abs_stage must be between 1 and 3")
    if abs(stage) > maximum_abs_stage:
        raise ValueError("stage exceeds configured limit")
    return f"{DRIVE_STAGE_TO_FIRMWARE[stage]:.2f}\n".encode("ascii")


def encode_steering_command(steering_deg, maximum_steering_deg):
    """Map a Lookahead steering target to the firmware steering protocol.

    An exact zero target uses the firmware's logical-center command.  This
    returns the accumulated steering drive position to zero without consulting
    the unreliable A0 neutral reference.  Nonzero targets remain proportional
    V commands.
    """
    steering_deg = float(steering_deg)
    maximum_steering_deg = float(maximum_steering_deg)
    if not math.isfinite(steering_deg):
        raise ValueError("steering_deg must be finite")
    if not math.isfinite(maximum_steering_deg) or maximum_steering_deg <= 0.0:
        raise ValueError("maximum_steering_deg must be positive and finite")
    if steering_deg == 0.0:
        return b"C\n"
    ratio = max(-1.0, min(1.0, steering_deg / maximum_steering_deg))
    return f"V,{ratio:.3f}\n".encode("ascii")


def encode_commands(stage, steering_deg, maximum_abs_stage, maximum_steering_deg):
    """Encode one drive line and one proportional-steering line."""
    return encode_drive_command(stage, maximum_abs_stage) + encode_steering_command(
        steering_deg, maximum_steering_deg
    )
