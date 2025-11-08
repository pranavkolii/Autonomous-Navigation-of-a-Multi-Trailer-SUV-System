#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import tty
import termios

LIN_VEL_STEP_SIZE = 0.5
ANG_VEL_STEP_SIZE = 0.1


class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_vel = 0.0
        self.steer_angle = 0.0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        instructions = """
Control Your SUV with Trailers!
---------------------------
Use keys:
    w: accelerate forward
    s: accelerate backward
    a: steer left
    d: steer right
    q: stop (emergency stop)
Esc to quit
"""
        self.get_logger().info(instructions)

        while True:
            key = self.getKey()

            if key == '\x1b':  # ESC to quit
                break
            elif key == 'q':  # emergency stop
                self.linear_vel = 0.0
                self.steer_angle = 0.0
            elif key == 'w':
                self.linear_vel += LIN_VEL_STEP_SIZE
            elif key == 's':
                self.linear_vel -= LIN_VEL_STEP_SIZE
            elif key == 'a':
                self.steer_angle += ANG_VEL_STEP_SIZE
            elif key == 'd':
                self.steer_angle -= ANG_VEL_STEP_SIZE

            # Clamp steer angle
            max_steer_angle = 1.0
            self.steer_angle = max(min(self.steer_angle, max_steer_angle), -max_steer_angle)
            # Clamp linear velocity
            max_linear_vel = 20.0
            self.linear_vel = max(min(self.linear_vel, max_linear_vel), -max_linear_vel)

            # Steering joints (LeftSteering_Joint, RightSteering_Joint)
            joint_positions = Float64MultiArray()
            joint_positions.data = [self.steer_angle] * 2
            self.joint_position_pub.publish(joint_positions)

            left_speed = 0
            right_speed = 0
            if self.steer_angle > 0.0: # turn left
                left_speed = -self.steer_angle * self.linear_vel / 3.0
            else:
                right_speed = self.steer_angle * self.linear_vel / 3.0

            # Wheel velocity joints (6 total in specific order)
            wheel_velocities = Float64MultiArray()
            wheel_velocities.data = [self.linear_vel + left_speed, self.linear_vel + right_speed, # front left, right 
                                     self.linear_vel + left_speed, self.linear_vel + right_speed, # rear left, right
                                     self.linear_vel + left_speed, self.linear_vel + right_speed, # trailer 1 left, right
                                     self.linear_vel + left_speed, self.linear_vel + right_speed] # trailer 1 left, right
            self.wheel_velocities_pub.publish(wheel_velocities)

            self.get_logger().info(
                f'Steer angle: {self.steer_angle:.2f}, Linear velocity: {self.linear_vel:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
