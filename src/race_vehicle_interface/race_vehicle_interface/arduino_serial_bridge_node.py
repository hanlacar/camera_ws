#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, Int64, String
from std_srvs.srv import SetBool

try:
    import serial
except ImportError:
    serial = None

from .serial_protocol import (
    encode_drive_command,
    encode_steering_command,
    parse_legacy_status,
    parse_legacy_steering_a0,
    parse_telemetry,
    steering_position_to_degrees,
)
from .command_mapping import camera_drive_to_stage, camera_wheel_to_steer


CAMERA_DRIVE_TOPIC = "/camera_drive"
CAMERA_WHEEL_TOPIC = "/camera_wheel"


class ArduinoSerialBridgeNode(Node):
    """Bridge ROS commands to the verified Arduino v14 text protocol."""

    def __init__(self):
        super().__init__("arduino_serial_bridge_node")
        defaults = {
            "port": "/dev/ttyACM0",
            "baudrate": 115200,
            "encoder_topic": "/inpulse",
            "rpm_topic": "/rpm",
            "steering_topic": "/steer_angle",
            "steering_position_topic": "/steer_position_ms",
            "steering_a0_topic": "/steer_a0",
            "steering_a0_error_topic": "/steer_a0_error",
            "steering_neutral_a0": 371,
            "steering_neutral_tolerance_a0": 5,
            "maximum_steering_position_ms": 440.0,
            "feedback_timeout_sec": 0.35,
            "reconnect_interval_sec": 1.0,
            "publish_raw_status": True,
            "dtr": True,
            "rts": True,
            "reset_on_connect": True,
            "reset_pulse_sec": 0.1,
            "write_timeout_sec": 0.1,
            "allow_transmit": False,
            "steering_only": False,
            "cmd_driving_topic": CAMERA_DRIVE_TOPIC,
            "cmd_steer_topic": CAMERA_WHEEL_TOPIC,
            "command_rate_hz": 20.0,
            "command_timeout_sec": 0.3,
            "firmware_heartbeat_sec": 0.5,
            "maximum_abs_stage": 0,
            "maximum_steering_deg": 27.0,
            "require_fresh_feedback": True,
            "legacy_status_poll_hz": 10.0,
            "status_poll_start_delay_sec": 2.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.serial_port = None
        self.receive_buffer = bytearray()
        self.last_open_attempt = 0.0
        self.last_telemetry = None
        self.last_reported_error = ""
        self.tx_enabled = False
        self.command_values = {}
        self.command_times = {}
        self.last_sent_stage = None
        self.last_sent_steer = None
        self.last_drive_write_time = None
        self.last_status_poll_time = None
        self.status_poll_logged = False
        self.serial_open_time = None
        self.last_logged_camera_stage = None
        self.last_logged_camera_steer = None

        self.encoder_pub = self.create_publisher(
            Int64, self.param("encoder_topic"), 10
        )
        self.rpm_pub = self.create_publisher(
            Float32, self.param("rpm_topic"), 10
        )
        self.steering_pub = self.create_publisher(
            Float32, self.param("steering_topic"), 10
        )
        self.steering_position_pub = self.create_publisher(
            Float32, self.param("steering_position_topic"), 10
        )
        self.steering_a0_pub = self.create_publisher(
            Int32, self.param("steering_a0_topic"), 10
        )
        self.steering_a0_error_pub = self.create_publisher(
            Int32, self.param("steering_a0_error_topic"), 10
        )
        self.steering_neutral_pub = self.create_publisher(
            Bool, "/steer_at_neutral", 10
        )
        self.connected_pub = self.create_publisher(
            Bool, "/arduino/connected", 10
        )
        self.valid_pub = self.create_publisher(
            Bool, "/arduino/feedback_valid", 10
        )
        self.status_pub = self.create_publisher(
            String, "/arduino/status", 10
        )
        self.raw_pub = self.create_publisher(
            String, "/arduino/raw_status", 10
        )

        self.create_subscription(
            Float32,
            self.param("cmd_driving_topic"),
            self.on_camera_drive,
            10,
        )
        self.create_subscription(
            Int32,
            self.param("cmd_steer_topic"),
            self.on_camera_wheel,
            10,
        )
        self.create_service(
            SetBool, "/arduino_bridge/set_tx_enabled", self.set_tx_enabled
        )

        self.create_timer(0.01, self.poll_serial)
        self.create_timer(0.1, self.publish_health)
        command_rate = max(1.0, float(self.param("command_rate_hz")))
        self.create_timer(1.0 / command_rate, self.transmit_command)
        self.get_logger().info(
            f"Arduino bridge: {self.param('port')} at "
            f"{self.param('baudrate')} baud; physical steering neutral "
            f"A0={self.param('steering_neutral_a0')}; command TX is LOCKED"
        )

    def param(self, name):
        return self.get_parameter(name).value

    def report(self, text, error=False):
        self.status_pub.publish(String(data=text))
        if error and text != self.last_reported_error:
            self.get_logger().error(text)
            self.last_reported_error = text

    def open_serial(self):
        if serial is None:
            self.report("pyserial_not_installed", error=True)
            return
        now = time.monotonic()
        if now - self.last_open_attempt < float(
            self.param("reconnect_interval_sec")
        ):
            return
        self.last_open_attempt = now
        try:
            port = serial.Serial()
            port.port = str(self.param("port"))
            port.baudrate = int(self.param("baudrate"))
            port.timeout = 0
            port.write_timeout = max(0.01, float(self.param("write_timeout_sec")))
            reset_on_connect = bool(self.param("reset_on_connect"))
            port.dtr = False if reset_on_connect else bool(self.param("dtr"))
            port.rts = False if reset_on_connect else bool(self.param("rts"))
            port.open()
            if reset_on_connect:
                time.sleep(max(0.01, float(self.param("reset_pulse_sec"))))
                port.dtr = bool(self.param("dtr"))
                port.rts = bool(self.param("rts"))
            self.serial_port = port
            self.receive_buffer.clear()
            self.last_sent_stage = None
            self.last_sent_steer = None
            self.last_drive_write_time = None
            self.last_status_poll_time = None
            self.serial_open_time = time.monotonic()
            self.last_encoder_count = None
            self.last_encoder_time = None
            self.last_reported_error = ""
            mode = "available_locked" if bool(self.param("allow_transmit")) else "disabled"
            self.report(f"serial_connected;command_tx_{mode}")
            self.get_logger().info(f"Opened Arduino serial port {port.port}")
        except (OSError, serial.SerialException) as exc:
            self.serial_port = None
            self.report(f"serial_open_failed:{exc}", error=True)

    def close_serial(self):
        port, self.serial_port = self.serial_port, None
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass

    def poll_serial(self):
        if self.serial_port is None:
            self.open_serial()
            return
        try:
            status_poll_hz = float(self.param("legacy_status_poll_hz"))
            now = time.monotonic()
            startup_complete = (
                self.serial_open_time is not None
                and now - self.serial_open_time
                >= float(self.param("status_poll_start_delay_sec"))
            )
            if status_poll_hz > 0.0 and startup_complete and (
                self.last_status_poll_time is None
                or now - self.last_status_poll_time >= 1.0 / status_poll_hz
            ):
                written = self.serial_port.write(b"S\n")
                if written == 2:
                    self.serial_port.flush()
                    self.last_status_poll_time = now
                    if not self.status_poll_logged:
                        self.get_logger().info("Legacy S status polling active")
                        self.status_poll_logged = True
            waiting = self.serial_port.in_waiting
            if waiting:
                self.receive_buffer.extend(self.serial_port.read(waiting))
            while b"\n" in self.receive_buffer:
                raw, _, remainder = self.receive_buffer.partition(b"\n")
                self.receive_buffer = bytearray(remainder)
                self.handle_line(raw.rstrip(b"\r"))
            if len(self.receive_buffer) > 4096:
                self.receive_buffer.clear()
                self.report("serial_line_too_long", error=True)
        except (OSError, serial.SerialException) as exc:
            self.report(f"serial_read_failed:{exc}", error=True)
            self.close_serial()

    def handle_line(self, raw):
        if bool(self.param("publish_raw_status")):
            text = raw.decode("utf-8", errors="replace").strip()
            if text:
                self.raw_pub.publish(String(data=text))
        telemetry = parse_telemetry(raw)
        steering_a0 = None
        if telemetry is None:
            telemetry = parse_legacy_status(raw)
            steering_a0 = parse_legacy_steering_a0(raw)
        if telemetry is None:
            return
        encoder_count, rpm, steering_position_ms = telemetry
        self.encoder_pub.publish(Int64(data=encoder_count))
        self.rpm_pub.publish(Float32(data=float(rpm)))
        if steering_position_ms is not None:
            steering_angle = steering_position_to_degrees(
                steering_position_ms,
                float(self.param("maximum_steering_position_ms")),
                float(self.param("maximum_steering_deg")),
            )
            self.steering_position_pub.publish(
                Float32(data=float(steering_position_ms))
            )
            self.steering_pub.publish(Float32(data=float(steering_angle)))
        if steering_a0 is not None:
            neutral_a0 = int(self.param("steering_neutral_a0"))
            error_a0 = int(steering_a0) - neutral_a0
            tolerance = max(0, int(self.param("steering_neutral_tolerance_a0")))
            self.steering_a0_pub.publish(Int32(data=int(steering_a0)))
            self.steering_a0_error_pub.publish(Int32(data=error_a0))
            self.steering_neutral_pub.publish(
                Bool(data=abs(error_a0) <= tolerance)
            )
        self.last_telemetry = time.monotonic()

    def update_command(self, name, value):
        if name == "steer" and not math.isfinite(value):
            return
        if name == "stage" and bool(self.param("steering_only")) and int(value) != 0:
            if self.tx_enabled:
                self.write_command(0, 0.0, force=True)
            self.tx_enabled = False
            self.report("command_tx_disarmed:nonzero_stage_in_steering_only_mode", error=True)
            return
        self.command_values[name] = value
        self.command_times[name] = time.monotonic()

    def reject_camera_command(self, reason):
        if self.tx_enabled:
            self.write_command(0, 0.0, force=True)
        self.tx_enabled = False
        self.command_values["stage"] = 0
        self.command_values["steer"] = 0.0
        self.command_times.clear()
        self.report(f"command_tx_disarmed:{reason}", error=True)

    def on_camera_drive(self, msg):
        stage, valid = camera_drive_to_stage(msg.data)
        if not valid:
            self.reject_camera_command("invalid_camera_drive")
            return
        self.update_command("stage", stage)
        if stage != self.last_logged_camera_stage:
            self.get_logger().info(
                f"Camera drive received: {msg.data:.1f} -> stage {stage}; "
                f"tx_enabled={self.tx_enabled}"
            )
            self.last_logged_camera_stage = stage

    def on_camera_wheel(self, msg):
        steer, valid = camera_wheel_to_steer(
            msg.data, int(self.param("maximum_steering_deg")))
        if not valid:
            self.reject_camera_command("invalid_camera_wheel")
            return
        self.update_command("steer", steer)
        if steer != self.last_logged_camera_steer:
            self.get_logger().info(
                f"Camera wheel received: {msg.data} -> {steer:.1f} deg; "
                f"tx_enabled={self.tx_enabled}"
            )
            self.last_logged_camera_steer = steer

    def feedback_is_fresh(self):
        return (
            self.last_telemetry is not None
            and time.monotonic() - self.last_telemetry
            <= float(self.param("feedback_timeout_sec"))
        )

    def set_tx_enabled(self, request, response):
        if not request.data:
            was_enabled = self.tx_enabled
            self.tx_enabled = False
            if was_enabled:
                self.write_command(0, 0.0, force=True)
            response.success = True
            response.message = "Arduino command TX disarmed"
            return response

        if not bool(self.param("allow_transmit")):
            response.success = False
            response.message = "refused: allow_transmit is false"
            return response
        maximum_abs_stage = int(self.param("maximum_abs_stage"))
        if not 1 <= maximum_abs_stage <= 3:
            response.success = False
            response.message = "refused: maximum_abs_stage must be 1..3"
            return response
        if bool(self.param("require_fresh_feedback")) and not self.feedback_is_fresh():
            response.success = False
            response.message = "refused: Arduino feedback is not fresh"
            return response

        self.tx_enabled = True
        response.success = True
        response.message = "Arduino command TX armed"
        return response

    def write_command(self, stage, steer, force=False):
        if self.serial_port is None or not self.serial_port.is_open:
            return False
        try:
            packets = []
            now = time.monotonic()
            heartbeat_sec = max(
                0.1, float(self.param("firmware_heartbeat_sec"))
            )
            heartbeat_due = (
                self.last_drive_write_time is None
                or now - self.last_drive_write_time >= heartbeat_sec
            )
            send_stage = (
                force or stage != self.last_sent_stage or heartbeat_due
            )
            send_steer = (
                force
                or self.last_sent_steer is None
                or abs(steer - self.last_sent_steer) >= 0.01
            )
            if send_stage:
                packets.append(
                    encode_drive_command(
                        stage, int(self.param("maximum_abs_stage"))
                    )
                )
            if send_steer:
                packets.append(
                    encode_steering_command(
                        steer, float(self.param("maximum_steering_deg"))
                    )
                )
            if not packets:
                return True
            packet = b"".join(packets)
            self.serial_port.write(packet)
            if send_stage:
                self.last_sent_stage = stage
                self.last_drive_write_time = now
            if send_steer:
                self.last_sent_steer = steer
            return True
        except (OSError, serial.SerialException, ValueError) as exc:
            self.report(f"serial_write_failed:{exc}", error=True)
            if not isinstance(exc, ValueError):
                self.close_serial()
            return False

    def transmit_command(self):
        if not self.tx_enabled:
            return

        now = time.monotonic()
        timeout = float(self.param("command_timeout_sec"))
        fresh = all(
            key in self.command_times and now - self.command_times[key] <= timeout
            for key in ("stage", "steer")
        )
        feedback_ok = (
            not bool(self.param("require_fresh_feedback"))
            or self.feedback_is_fresh()
        )
        if not fresh or not feedback_ok:
            self.write_command(0, 0.0, force=True)
            self.tx_enabled = False
            reason = "stale_command" if not fresh else "stale_feedback"
            self.report(f"command_tx_disarmed:{reason}", error=True)
            return

        stage = int(self.command_values["stage"])
        steer = float(self.command_values["steer"])
        if bool(self.param("steering_only")) and stage != 0:
            self.write_command(0, 0.0, force=True)
            self.tx_enabled = False
            self.report("command_tx_disarmed:nonzero_stage_in_steering_only_mode", error=True)
            return
        maximum_abs_stage = int(self.param("maximum_abs_stage"))
        if abs(stage) > maximum_abs_stage:
            self.write_command(0, 0.0, force=True)
            self.tx_enabled = False
            self.report("command_tx_disarmed:stage_out_of_range", error=True)
            return
        self.write_command(stage, steer)

    def publish_health(self):
        connected = self.serial_port is not None and self.serial_port.is_open
        timeout = float(self.param("feedback_timeout_sec"))
        valid = (
            connected
            and self.last_telemetry is not None
            and time.monotonic() - self.last_telemetry <= timeout
        )
        self.connected_pub.publish(Bool(data=connected))
        self.valid_pub.publish(Bool(data=valid))
        if connected and not valid:
            self.status_pub.publish(String(data="waiting_for_fresh_telemetry"))

    def destroy_node(self):
        if self.tx_enabled:
            self.write_command(0, 0.0, force=True)
        self.tx_enabled = False
        self.close_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # ros2 launch and the terminal can deliver overlapping SIGINTs.
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
