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
        self.declare_parameter('head1_x', 0.65)
        self.declare_parameter('head1_z', 0.16)
        self.declare_parameter('head2_x', 0.65)
        self.declare_parameter('head2_z', 0.16)
        self.declare_parameter('y_min', -0.9)
        self.declare_parameter('y_max', 0.9)
        self.declare_parameter('speed', 0.08)  # m/s
        self.declare_parameter('update_hz', 20.0)

        self.world_name = self.get_parameter('world_name').value
        self.models = self.get_parameter('models').value
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.speed = float(self.get_parameter('speed').value)
        self.direction = 1.0

        # fixed (x, z) per model; y is animated
        self.base_pose = {}
        for name in self.models:
            if name.startswith('head'):
                idx = name[-1]
                self.base_pose[name] = (
                    float(self.get_parameter('head%s_x' % idx).value),
                    float(self.get_parameter('head%s_z' % idx).value),
                )
            elif name.startswith('marker'):
                idx = name[-1]
                self.base_pose[name] = (
                    float(self.get_parameter('head%s_x' % idx).value),
                    float(self.get_parameter('head%s_z' % idx).value) + 0.12,
                )

        # initial y spread across the conveyor
        self.y = {}
        n = max(1, len(self.models) - 1)
        for i, name in enumerate(self.models):
            self.y[name] = self.y_min + (self.y_max - self.y_min) * i / n

        period = 1.0 / float(self.get_parameter('update_hz').value)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(
            'conveyor started: moving %s along Y [%s, %s]' %
            (self.models, self.y_min, self.y_max))

    def set_pose(self, name, x, y, z):
        req = ("name: '%s' pose: { position: { x: %f y: %f z: %f } "
               "orientation: { x: 0 y: 0 z: 0 w: 1 } }" % (name, x, y, z))
        cmd = [
            'ign', 'service', '-s', '/world/%s/set_pose' % self.world_name,
            '-reqtype', 'ignition.msgs.Pose',
            '-reptype', 'ignition.msgs.Boolean',
            '-timeout', '2000', '--', req,
        ]
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
        except Exception as e:
            self.get_logger().warn('set_pose failed for %s: %s' % (name, e))

    def timer_cb(self):
        dt = 0.05
        for name in self.models:
            self.y[name] += self.direction * self.speed * dt
        if self.direction > 0 and any(self.y[n] >= self.y_max for n in self.models):
            self.direction = -1.0
            for name in self.models:
                self.y[name] = min(self.y[name], self.y_max)
        elif self.direction < 0 and any(self.y[n] <= self.y_min for n in self.models):
            self.direction = 1.0
            for name in self.models:
                self.y[name] = max(self.y[name], self.y_min)
        for name in self.models:
            x, z = self.base_pose[name]
            self.set_pose(name, x, self.y[name], z)


def main():
    rclpy.init()
    node = Conveyor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
