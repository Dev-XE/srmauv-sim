#!/usr/bin/env python3
"""Depth sensor for the SRM AUV (pressure-depth below the water surface).

Gazebo Classic has no pressure sensor, so depth is derived from the vehicle's
Z (ground-truth odometry) as a real depth sensor would report it:
    depth = water_surface_z - z   (metres below surface, >= 0)
Small Gaussian noise is added for realism.

Subscribes: odom  (nav_msgs/Odometry)
Publishes:  depth (std_msgs/Float64)     -- metres below surface
            pose_depth (geometry_msgs/PoseWithCovarianceStamped) optional? no.
"""
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

WATER_SURFACE_Z = 2.5   # pool water surface height in world frame (m)
NOISE_STDDEV = 0.01     # ~1 cm depth noise


class DepthSensor(Node):
    def __init__(self):
        super().__init__('depth_sensor')
        self.declare_parameter('water_surface_z', WATER_SURFACE_Z)
        self.surface = self.get_parameter('water_surface_z').value
        self.pub = self.create_publisher(Float64, 'depth', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.get_logger().info(
            f'depth sensor up: depth = {self.surface} - z (+noise)')

    def on_odom(self, m):
        z = m.pose.pose.position.z
        depth = self.surface - z + random.gauss(0.0, NOISE_STDDEV)
        msg = Float64()
        msg.data = float(max(0.0, depth))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DepthSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
