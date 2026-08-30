#!/usr/bin/env python3

import math
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class RaceBBlinkingObstacle(Node):
    EXPECTED_MODEL_NAME = 'race_B_obstacle_dynamic'
    FORBIDDEN_MODEL_NAMES = {'m2wr'}

    def __init__(self):
        super().__init__('race_b_blinking_obstacle')

        package_share = get_package_share_directory('gazebo_car_description')
        default_sdf = f'{package_share}/models/race_B_obstacle_dynamic/model.sdf'

        self.declare_parameter('world_name', 'default')
        self.declare_parameter('model_name', 'race_B_obstacle_dynamic')
        self.declare_parameter('model_sdf', default_sdf)

        # 장애물 위치
        self.declare_parameter('x', -6.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.195)
        self.declare_parameter('yaw', 1.5707963267948966)

        # 요구 동작
        self.declare_parameter('hover_height', 1.0)
        self.declare_parameter('hover_duration_sec', 5.0)
        self.declare_parameter('drop_duration_sec', 1.0)
        self.declare_parameter('rise_duration_sec', 1.0)
        self.declare_parameter('ground_duration_sec', 5.0)
        self.declare_parameter('update_period_sec', 0.05)
        self.declare_parameter('retry_period_sec', 0.5)

        self.world_name = str(self.get_parameter('world_name').value)
        self.model_name = str(self.get_parameter('model_name').value)
        self.model_sdf = str(self.get_parameter('model_sdf').value)

        if self.model_name in self.FORBIDDEN_MODEL_NAMES:
            raise RuntimeError(
                'race_b_blinking_obstacle refuses to control vehicle model '
                f'"{self.model_name}". Use "{self.EXPECTED_MODEL_NAME}".'
            )

        if self.model_name != self.EXPECTED_MODEL_NAME:
            raise RuntimeError(
                'race_b_blinking_obstacle must target '
                f'"{self.EXPECTED_MODEL_NAME}", got "{self.model_name}".'
            )

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.ground_z = float(self.get_parameter('z').value)
        self.yaw = float(self.get_parameter('yaw').value)

        self.hover_height = float(self.get_parameter('hover_height').value)
        self.hover_duration_sec = float(self.get_parameter('hover_duration_sec').value)
        self.drop_duration_sec = max(0.0, float(self.get_parameter('drop_duration_sec').value))
        self.rise_duration_sec = max(0.0, float(self.get_parameter('rise_duration_sec').value))
        self.ground_duration_sec = float(self.get_parameter('ground_duration_sec').value)

        self.update_period_sec = max(0.02, float(self.get_parameter('update_period_sec').value))
        self.retry_period_sec = max(0.1, float(self.get_parameter('retry_period_sec').value))

        self.hover_z = self.ground_z + self.hover_height

        self._phase = 'ensure_hover'
        self._phase_start_time = time.monotonic()
        self._next_action_time = self._phase_start_time
        self._last_warning_time = {}

        self._timer = self.create_timer(self.update_period_sec, self._tick)

        self.get_logger().info(
            'Race B obstacle ready: '
            f'{self.model_name} | x={self.x:.3f}, y={self.y:.3f}, '
            f'ground_z={self.ground_z:.3f}, hover_z={self.hover_z:.3f} | '
            f'sequence=hover {self.hover_duration_sec:.1f}s '
            f'-> drop {self.drop_duration_sec:.1f}s '
            f'-> ground {self.ground_duration_sec:.1f}s '
            f'-> rise {self.rise_duration_sec:.1f}s '
            '-> hover repeat'
        )

    def _tick(self):
        now = time.monotonic()

        if now < self._next_action_time:
            return

        if self._phase == 'ensure_hover':
            self._ensure_hover(now)
        elif self._phase == 'hover':
            self._update_hover(now)
        elif self._phase == 'drop':
            self._update_drop(now)
        elif self._phase == 'ground':
            self._update_ground(now)
        elif self._phase == 'rise':
            self._update_rise(now)

    def _ensure_hover(self, now):
        if not self._model_exists():
            if not self._spawn_at(self.hover_z):
                self._next_action_time = now + self.retry_period_sec
                return
        else:
            if not self._set_pose(self.hover_z, 'hover'):
                self._next_action_time = now + self.retry_period_sec
                return

        phase_time = time.monotonic()
        self._enter_phase(
            'hover',
            phase_time,
            self.hover_duration_sec,
            f'{self.model_name}: 1m 상공 대기 시작'
        )

    def _update_hover(self, now):
        if now - self._phase_start_time < self.hover_duration_sec:
            self._next_action_time = now + self.update_period_sec
            return

        self._enter_phase(
            'drop',
            now,
            0.0,
            f'{self.model_name}: 낙하 시작'
        )

    def _update_drop(self, now):
        if self.drop_duration_sec <= 0.0:
            progress = 1.0
        else:
            progress = min(1.0, (now - self._phase_start_time) / self.drop_duration_sec)

        # 부드러운 낙하 보간
        smooth = progress * progress * (3.0 - 2.0 * progress)
        current_z = self.hover_z + (self.ground_z - self.hover_z) * smooth

        if not self._set_pose(current_z, 'drop'):
            self._next_action_time = now + self.retry_period_sec
            return

        if progress >= 1.0:
            phase_time = time.monotonic()
            self._enter_phase(
                'ground',
                phase_time,
                self.ground_duration_sec,
                f'{self.model_name}: 바닥 대기 시작'
            )
        else:
            self._next_action_time = now + self.update_period_sec

    def _update_ground(self, now):
        if now - self._phase_start_time < self.ground_duration_sec:
            self._next_action_time = now + self.update_period_sec
            return

        # 사라지지 않고 다시 1m 상공으로 이동
        self._enter_phase(
            'rise',
            now,
            0.0,
            f'{self.model_name}: 상승 시작'
        )

    def _update_rise(self, now):
        if self.rise_duration_sec <= 0.0:
            progress = 1.0
        else:
            progress = min(1.0, (now - self._phase_start_time) / self.rise_duration_sec)

        smooth = progress * progress * (3.0 - 2.0 * progress)
        current_z = self.ground_z + (self.hover_z - self.ground_z) * smooth

        if not self._set_pose(current_z, 'rise'):
            self._next_action_time = now + self.retry_period_sec
            return

        if progress >= 1.0:
            phase_time = time.monotonic()
            self._enter_phase(
                'hover',
                phase_time,
                self.hover_duration_sec,
                f'{self.model_name}: 다시 1m 상공 대기 시작'
            )
        else:
            self._next_action_time = now + self.update_period_sec

    def _enter_phase(self, phase, now, wait_sec, log_message=None):
        self._phase = phase
        self._phase_start_time = now
        self._next_action_time = now + max(0.0, wait_sec)

        if log_message:
            self.get_logger().info(log_message)

    def _spawn_at(self, z):
        cmd = [
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-world',
            self.world_name,
            '-file',
            self.model_sdf,
            '-name',
            self.model_name,
            '-allow_renaming',
            'false',
            '-x',
            f'{self.x:.6f}',
            '-y',
            f'{self.y:.6f}',
            '-z',
            f'{z:.6f}',
            '-Y',
            f'{self.yaw:.6f}',
        ]

        if self._run_command(cmd, 'spawn', timeout_sec=6.0):
            self.get_logger().info(
                f'{self.model_name}: 1m 상공에 생성 완료 z={z:.3f}'
            )
            return True

        if self._model_exists():
            return self._set_pose(z, 'hover')

        self._warn_throttled(
            'spawn_failed',
            f'{self.model_name}: 생성 실패, 재시도 중'
        )
        return False

    def _set_pose(self, z, label):
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        request = (
            f'name: "{self.model_name}" '
            f'position {{ x: {self.x:.6f} y: {self.y:.6f} z: {z:.6f} }} '
            f'orientation {{ x: 0.0 y: 0.0 z: {qz:.9f} w: {qw:.9f} }}'
        )

        cmd = [
            'gz',
            'service',
            '-s',
            f'/world/{self.world_name}/set_pose',
            '--reqtype',
            'gz.msgs.Pose',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '1000',
            '--req',
            request,
        ]

        success, output = self._run_command_with_output(cmd, f'set_pose:{label}', 2.0)

        if not success:
            self._warn_throttled(
                f'set_pose_{label}',
                f'{self.model_name}: {label} 위치 이동 실패'
                + (f' {output}' if output else '')
            )
            return False

        if 'data: false' in output.lower():
            self._warn_throttled(
                f'set_pose_{label}_false',
                f'{self.model_name}: Gazebo가 {label} 위치 이동을 거부함'
            )
            return False

        return True

    def _model_exists(self):
        try:
            result = subprocess.run(
                ['gz', 'model', '--list'],
                check=False,
                capture_output=True,
                text=True,
                timeout=4.0,
            )
        except (subprocess.TimeoutExpired, OSError):
            return False

        if result.returncode != 0:
            return False

        for line in result.stdout.splitlines():
            line = line.strip()
            if line.startswith('- ') and line[2:].strip() == self.model_name:
                return True

        return False

    def _run_command(self, cmd, action, timeout_sec):
        success, _ = self._run_command_with_output(cmd, action, timeout_sec)
        return success

    def _run_command_with_output(self, cmd, action, timeout_sec):
        try:
            result = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
        except subprocess.TimeoutExpired:
            return False, f'{action} command timed out.'
        except OSError as exc:
            return False, f'Could not start {action} command: {exc}'

        output = (result.stdout + result.stderr).strip()

        if result.returncode == 0:
            return True, output

        return False, (
            output + f' {action} command exited with code {result.returncode}.'
        ).strip()

    def _warn_throttled(self, key, message, period_sec=5.0):
        now = time.monotonic()
        last = self._last_warning_time.get(key)

        if last is not None and now - last < period_sec:
            return

        self._last_warning_time[key] = now
        self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = RaceBBlinkingObstacle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
