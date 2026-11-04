#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import math

class FakeWrenchPublisher(Node):
    def __init__(self):
        super().__init__('fake_wrench_publisher')
        self.pub = self.create_publisher(WrenchStamped, '/sensor_joint/force_torque/fake', 10)
        self.timer = self.create_timer(0.01, self.tick)  # match your control rate
        self.t0 = self.get_clock().now()
        self.F_mag = 10.0   # N
        self.t_start = 10.0
        self.duration = 3.0

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "fr3_ft_sensor_link"
        msg.wrench.force.y = self.step_function(t, 5, 2, 1.0)
        # torque left at zero (default)
        self.pub.publish(msg)

    def step_function(self, t, t_interval, duration, F_mag):
        if t % (t_interval + duration) < duration:
            return F_mag
        else:
            return 0.0


def main():
    rclpy.init()
    rclpy.spin(FakeWrenchPublisher())

if __name__ == '__main__':
    main()