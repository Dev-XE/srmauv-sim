#!/usr/bin/env python3
"""Keyboard teleop for the SRM AUV. Publishes geometry_msgs/Twist to cmd_vel.

Controls (hold/tap):
  w / s : surge forward / back   (linear.x)
  a / d : yaw left / right        (angular.z)
  q / e : sway left / right       (linear.y)
  r / f : heave up / down         (linear.z)
  i / k : pitch up / down         (angular.y)
  j / l : roll  left / right      (angular.x)
  space : stop (zero all)
  Ctrl-C: quit
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

STEP = 0.5
BINDINGS = {
    'w': ('linear', 'x', +STEP), 's': ('linear', 'x', -STEP),
    'q': ('linear', 'y', +STEP), 'e': ('linear', 'y', -STEP),
    'r': ('linear', 'z', +STEP), 'f': ('linear', 'z', -STEP),
    'j': ('angular', 'x', +STEP), 'l': ('angular', 'x', -STEP),
    'i': ('angular', 'y', +STEP), 'k': ('angular', 'y', -STEP),
    'a': ('angular', 'z', +STEP), 'd': ('angular', 'z', -STEP),
}


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = Node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    twist = Twist()
    print(__doc__)
    try:
        while rclpy.ok():
            k = get_key()
            if k == '\x03':            # Ctrl-C
                break
            if k == ' ':
                twist = Twist()
            elif k in BINDINGS:
                grp, axis, val = BINDINGS[k]
                cur = getattr(getattr(twist, grp), axis)
                setattr(getattr(twist, grp), axis, max(-1.0, min(1.0, cur + val)))
            pub.publish(twist)
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())           # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
