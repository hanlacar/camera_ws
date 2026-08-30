#!/usr/bin/env python3

import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class RaceBSpawnM2WROnce(Node):
    EXPECTED_ROBOT_NAME = 'm2wr'
    RESERVED_OBSTACLE_NAMES = {'race_B_obstacle_dynamic'}

    def __init__(self):
        super().__init__('race_b_spawn_m2wr_once')

        package_share = get_package_share_directory('gazebo_car_description')
        default_urdf = f'{package_share}/urdf/m2wr_gz.urdf'

        self.declare_parameter('world_name', 'default')
        self.declare_parameter('robot_name', 'm2wr')
        self.declare_parameter('urdf_file', default_urdf)
        self.declare_parameter('x', '3.0')
        self.declare_parameter('y', '-3.0')
        self.declare_parameter('z', '0.25')
        self.declare_parameter('yaw', '3.141592653589793')
        self.declare_parameter('retry_period_sec', 0.5)
        self.declare_parameter('timeout_sec', 20.0)

        self.world_name = str(self.get_parameter('world_name').value)
        self.robot_name = str(self.get_parameter('robot_name').value)
        self.urdf_file = str(self.get_parameter('urdf_file').value)

        if self.robot_name in self.RESERVED_OBSTACLE_NAMES:
            raise RuntimeError(
                'race_b_spawn_m2wr_once refuses to spawn the vehicle with '
                f'obstacle name "{self.robot_name}". Use "{self.EXPECTED_ROBOT_NAME}".'
            )

        if self.robot_name != self.EXPECTED_ROBOT_NAME:
            raise RuntimeError(
                'race_b_spawn_m2wr_once must spawn '
                f'"{self.EXPECTED_ROBOT_NAME}", got "{self.robot_name}".'
            )

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.retry_period_sec = float(self.get_parameter('retry_period_sec').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.done = False
        self._start_time = time.monotonic()
        self._next_action_time = self._start_time
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'Race B guarded spawn ready for {self.robot_name} at '
            f'({self.x:.3f}, {self.y:.3f}, {self.z:.3f}), yaw={self.yaw:.3f}'
        )

    def _tick(self):
        if self.done:
            return

        now = time.monotonic()
        if now < self._next_action_time:
            return

        if now - self._start_time > self.timeout_sec:
            self.get_logger().error(
                f'Timed out waiting to spawn or find {self.robot_name}.'
            )
            self.done = True
            return

        models = self._list_models()
        if models is None:
            self._next_action_time = now + self.retry_period_sec
            return

        if self.robot_name in models:
            self.get_logger().info(
                f'{self.robot_name} already exists; skipping spawn to keep one vehicle.'
            )
            self.done = True
            return

        if self._spawn_robot():
            self.get_logger().info(f'Spawned {self.robot_name}')
            self.done = True
            return

        # If the create command raced with an existing entity, do one final
        # model-list check before deciding to retry.
        models = self._list_models()
        if models is not None and self.robot_name in models:
            self.get_logger().info(
                f'{self.robot_name} exists after spawn attempt; not spawning again.'
            )
            self.done = True
            return

        self._next_action_time = now + self.retry_period_sec

    def _spawn_robot(self):
        cmd = [
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-world',
            self.world_name,
            '-file',
            self.urdf_file,
            '-name',
            self.robot_name,
            '-allow_renaming',
            'false',
            '-x',
            f'{self.x:.6f}',
            '-y',
            f'{self.y:.6f}',
            '-z',
            f'{self.z:.6f}',
            '-Y',
            f'{self.yaw:.6f}',
        ]
        return self._run_command(cmd, 'spawn')

    def _list_models(self):
        result = self._run_command(['gz', 'model', '--list'], 'list models', output=True)
        if result is None:
            return None

        models = set()
        for line in result.splitlines():
            text = line.strip()
            if text.startswith('- '):
                models.add(text[2:].strip())
        return models

    def _run_command(self, cmd, action, output=False):
        try:
            result = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=6.0,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().debug(f'Robot {action} command timed out.')
            return None if output else False
        except OSError as exc:
            self.get_logger().warn(f'Could not start robot {action} command: {exc}')
            return None if output else False

        combined_output = (result.stdout + result.stderr).strip()
        if result.returncode == 0:
            if output:
                return result.stdout
            if combined_output:
                self.get_logger().debug(combined_output)
            return True

        if combined_output:
            self.get_logger().debug(combined_output)
        return None if output else False


def main(args=None):
    rclpy.init(args=args)
    node = RaceBSpawnM2WROnce()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
