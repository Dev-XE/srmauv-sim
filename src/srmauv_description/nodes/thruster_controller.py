#!/usr/bin/env python3
"""8-thruster allocation controller for the SRM AUV (SEDNA-style, 6-DOF).

Subscribes:  cmd_vel  (geometry_msgs/Twist)  -- per-DOF effort demand in [-1, 1]
Publishes:   wrench   (geometry_msgs/Wrench)  -- net body wrench to the
             libgazebo_ros_force plugin (applied in the link/body frame)

Thruster layout (positions relative to center of mass, body frame):
  4 vertical  -> heave / roll / pitch
  2 surge (port+stbd, +X, offset in Y) -> surge + yaw
  2 sway  (fore+aft,  +Y, offset in X) -> sway  + yaw
This is full rank 6, so all six DOF are controllable. Positions can be
refined to the exact CAD thruster mounts later without changing the logic.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench

# (position [m] relative to CoM, unit thrust direction) per thruster
THRUSTERS = [
    ((0.20, 0.25, 0.0), (0.0, 0.0, 1.0)),    # V1 vertical
    ((0.20, -0.25, 0.0), (0.0, 0.0, 1.0)),   # V2
    ((-0.20, 0.25, 0.0), (0.0, 0.0, 1.0)),   # V3
    ((-0.20, -0.25, 0.0), (0.0, 0.0, 1.0)),  # V4
    ((0.0, 0.25, 0.05), (1.0, 0.0, 0.0)),    # H1 surge (port)
    ((0.0, -0.25, 0.05), (1.0, 0.0, 0.0)),   # H2 surge (stbd)
    ((0.25, 0.0, 0.05), (0.0, 1.0, 0.0)),    # H3 sway (fore)
    ((-0.25, 0.0, 0.05), (0.0, 1.0, 0.0)),   # H4 sway (aft)
]
T_MAX = 40.0       # max thrust per thruster [N] (~Blue Robotics T200 class)
FORCE_SCALE = 70.0  # N per unit linear cmd
TORQUE_SCALE = 12.0  # N·m per unit angular cmd


def allocation_matrix():
    cols = []
    for r, d in THRUSTERS:
        r = np.array(r, float)
        d = np.array(d, float)
        cols.append(np.concatenate([d, np.cross(r, d)]))
    return np.array(cols).T          # 6 x 8


class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        self.A = allocation_matrix()
        self.Ainv = np.linalg.pinv(self.A)
        self.cmd = np.zeros(6)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.pub = self.create_publisher(Wrench, 'wrench', 10)
        self.create_timer(0.02, self.update)        # 50 Hz
        self.get_logger().info(
            f'thruster_controller up: 8 thrusters, allocation rank '
            f'{np.linalg.matrix_rank(self.A)}/6')

    def on_cmd(self, m):
        self.cmd = np.array([
            FORCE_SCALE * m.linear.x, FORCE_SCALE * m.linear.y, FORCE_SCALE * m.linear.z,
            TORQUE_SCALE * m.angular.x, TORQUE_SCALE * m.angular.y, TORQUE_SCALE * m.angular.z])

    def update(self):
        thrusts = np.clip(self.Ainv @ self.cmd, -T_MAX, T_MAX)
        w = self.A @ thrusts                          # achieved body wrench
        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = map(float, w[0:3])
        msg.torque.x, msg.torque.y, msg.torque.z = map(float, w[3:6])
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
