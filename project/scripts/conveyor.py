#!/usr/bin/env python3
"""Animate the conveyor: slide the head models (and their ArUco markers)
back and forth along the conveyor (Y axis) by setting their pose in Gazebo
via `ign service ... /world/<world>/set_pose`.
"""
import rclpy
import subprocess
from rclpy.node import Node


class Conveyor(Node):
    def __init__(self):
        super().__init__('conveyor')
        self.declare_parameter('world_name', 'project_world')
        self.declare_parameter('models', ['head1', 'marker1', 'head2', 'marker2'])
        self.declare_parameter('head1_x', 0.5)
        self.declare_parameter('head1_z', 1.0)
        self.declare_parameter('head2_x', 0.5)
        self.declare_parameter('head2_z', 1.0)
        self.declare_parameter('y_min', -0.25)
        self.declare_parameter('y_max', 0.25)
        self.declare_parameter('speed', 0.02)  # m/s
        self.declare_parameter('update_hz', 20.0)

        self.world_name = self.get_parameter('world_name').value
        self.models = self.get_parameter('models').value
        if len(self.models) % 2 != 0:
            raise ValueError('models must list each head followed by its marker')
        self.model_pairs = list(zip(self.models[::2], self.models[1::2]))
        if any(not head.startswith('head') or not marker.startswith('marker')
               for head, marker in self.model_pairs):
            raise ValueError('models must contain alternating head/marker names')
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.speed = float(self.get_parameter('speed').value)
        self.dt = 1.0 / float(self.get_parameter('update_hz').value)
        self.direction = 1.0

        # Keep each marker at the same conveyor position as its head.
        self.base_pose = {}
        for head, marker in self.model_pairs:
            idx = head[len('head'):]
            x = float(self.get_parameter('head%s_x' % idx).value)
            z = float(self.get_parameter('head%s_z' % idx).value)
            self.base_pose[head] = (x, z)
            # The head top is 0.22 m above its origin; leave 1 cm below the tag.
            self.base_pose[marker] = (x, z + 0.23)

        # Spread heads along the belt, with each tag sharing its head's y.
        self.y = {}
        n = max(1, len(self.model_pairs) - 1)
        for i, (head, _) in enumerate(self.model_pairs):
            self.y[head] = self.y_min + (self.y_max - self.y_min) * i / n

        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info(
            'conveyor started: moving %s along Y [%s, %s]' %
            (self.models, self.y_min, self.y_max))

    def set_poses(self, poses):
        req = ' '.join(
            "pose: { name: '%s' position: { x: %f y: %f z: %f } "
            "orientation: { x: 0 y: 0 z: 0 w: 1 } }" % (name, x, y, z)
            for name, x, y, z in poses
        )
        cmd = [
            'ign', 'service', '-s', '/world/%s/set_pose_vector' % self.world_name,
            '--reqtype', 'ignition.msgs.Pose_V',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000', '-r', req,
        ]
        try:
            result = subprocess.run(
                cmd, check=True, capture_output=True, text=True, timeout=3)
            if 'data: true' not in result.stdout:
                raise RuntimeError('Gazebo rejected the pose update: %s' % result.stdout.strip())
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, RuntimeError) as e:
            self.get_logger().error(
                'Failed to update conveyor poses: %s' % e,
                throttle_duration_sec=5.0)

    def timer_cb(self):
        for head, _ in self.model_pairs:
            self.y[head] += self.direction * self.speed * self.dt
        if self.direction > 0 and any(self.y[head] >= self.y_max for head, _ in self.model_pairs):
            self.direction = -1.0
            for head, _ in self.model_pairs:
                self.y[head] = min(self.y[head], self.y_max)
        elif self.direction < 0 and any(self.y[head] <= self.y_min for head, _ in self.model_pairs):
            self.direction = 1.0
            for head, _ in self.model_pairs:
                self.y[head] = max(self.y[head], self.y_min)
        poses = []
        for head, marker in self.model_pairs:
            x, z = self.base_pose[head]
            poses.append((head, x, self.y[head], z))
            x, z = self.base_pose[marker]
            poses.append((marker, x, self.y[head], z))
        self.set_poses(poses)


def main():
    rclpy.init()
    node = Conveyor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
