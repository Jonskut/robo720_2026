#!/usr/bin/env python3
"""Publish a static ObstacleArray (latched) describing the obstacles on the conveyor.

The controller latches this message (transient_local) to learn each obstacle's
name, shape and dimensions. Live positions are then updated over TF from Gazebo
(world/<world>/pose/info), exactly as in exercise 6.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from custom_msgs.msg import ObstacleArray, Obstacle
from geometry_msgs.msg import Point, Quaternion


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.declare_parameter('obstacle_name', 'obstacle_big')
        self.declare_parameter('obstacle_shape', 1)          # 1 = BOX
        self.declare_parameter('obstacle_x', 0.5)
        self.declare_parameter('obstacle_y', -0.65)
        self.declare_parameter('obstacle_z', 1.4)
        self.declare_parameter('obstacle_sx', 0.5)
        self.declare_parameter('obstacle_sy', 0.5)
        self.declare_parameter('obstacle_sz', 0.8)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(ObstacleArray, '/obstacles', latched)

    def publish_obstacles(self):
        msg = ObstacleArray()
        obs = Obstacle()
        obs.name = self.get_parameter('obstacle_name').value
        obs.shape_type = int(self.get_parameter('obstacle_shape').value)
        obs.position = Point(
            x=self.get_parameter('obstacle_x').value,
            y=self.get_parameter('obstacle_y').value,
            z=self.get_parameter('obstacle_z').value,
        )
        obs.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        if obs.shape_type == Obstacle.BOX:
            obs.box_size = [
                float(self.get_parameter('obstacle_sx').value),
                float(self.get_parameter('obstacle_sy').value),
                float(self.get_parameter('obstacle_sz').value),
            ]
        else:
            obs.radius = 0.1
        msg.obstacles = [obs]
        self.pub.publish(msg)
        self.get_logger().info('Published obstacle array: %s (%d)' % (obs.name, obs.shape_type), throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = ObstaclePublisher()
    # Republish periodically so late-joining subscribers (e.g. the lifecycle
    # controller) reliably receive the obstacle shape/dimension data. Live
    # positions still come over TF from Gazebo (ex6 approach).
    node.create_timer(1.0, node.publish_obstacles)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
