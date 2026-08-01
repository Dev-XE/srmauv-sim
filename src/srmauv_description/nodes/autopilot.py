#!/usr/bin/env python3
"""Waypoint-following autopilot for the SRM AUV.

Line-of-sight guidance: reads /srmauv/odom, steers toward each course waypoint
(turn to face it, drive forward when aligned, hold target depth), and publishes
/srmauv/cmd_vel to the 8-thruster controller. Stops when the course is done.

Run ONE of {autopilot, teleop} at a time (both drive cmd_vel).
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# NIOT course waypoints (x, y, z_depth, label) — matches the arena layout.
# Bins branch (marker drop) — no torpedo needed.
WAYPOINTS = [
    (-7.5, 0.34, 1.6, "red flower  (touch buoy, alongside)"),
    (-6.2, 0.75, 1.6, "veer out + forward, disengage the pole"),  # move AWAY, not along
    (-1.5, 0.0, 1.3, "L'ove Lane gate  (pass over)"),
    (3.68, 3.0, 1.3, "bins  (drop marker)"),
    (8.0, 5.5, 2.2, "octagon  (surface)"),
]
ARRIVE_XY = 0.7      # m: horizontal arrival radius
K_YAW = 1.2          # yaw P-gain
K_Z = 1.2            # depth P-gain
CRUISE = 1.0         # forward effort when aligned
ALIGN_TOL = 1.0      # rad: only drive forward if |yaw error| < this


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


class Autopilot(Node):
    def __init__(self):
        super().__init__('autopilot')
        self.pose = None
        self.i = 0
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.update)     # 10 Hz
        self.get_logger().info(f'autopilot: following {len(WAYPOINTS)} waypoints')

    def on_odom(self, m):
        self.pose = m.pose.pose

    def update(self):
        cmd = Twist()
        if self.pose is None:
            return
        if self.i >= len(WAYPOINTS):
            self.pub.publish(cmd)                # course complete -> hold
            return

        tx, ty, tz, label = WAYPOINTS[self.i]
        p = self.pose.position
        yaw = yaw_from_quat(self.pose.orientation)
        dx, dy = tx - p.x, ty - p.y
        dist = math.hypot(dx, dy)

        if dist < ARRIVE_XY:
            self.get_logger().info(f'reached WP{self.i}: {label}')
            self.i += 1
            self.pub.publish(cmd)
            return

        yaw_err = wrap(math.atan2(dy, dx) - yaw)
        cmd.angular.z = clamp(K_YAW * yaw_err)
        cmd.linear.x = CRUISE * max(0.0, math.cos(yaw_err)) if abs(yaw_err) < ALIGN_TOL else 0.0
        cmd.linear.z = clamp(K_Z * (tz - p.z))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = Autopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
