# Teleop\_node.py (unmodified from assignment instructions)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys


class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('autonomous_control_node')

        self.joint_position_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        # self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pose_sub = self.create_subscription(
            Pose, '/autonomous_controller/pose', self.pose_callback, 10)

        self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0

        # self.settings = termios.tcgetattr(sys.stdin)

    """def getKey(self):
       tty.setraw(sys.stdin.fileno())
       rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
       if rlist:
           key = sys.stdin.read(1)
       else:
           key = ''

       termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
       return key"""

    def pose_callback(self, msg: Pose):
        self.robot_x = msg.position.x
        self.robot_y = msg.position.y

        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.robot_yaw = yaw

        self.get_logger().info("Got the position of the robot: " + str(self.robot_x) +
                               " " + str(self.robot_y) + " " + str(self.robot_yaw))


    """def run_keyboard_control(self):
        self.msg = '''
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d
        q : force stop
        Esc to quit
        '''

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel = 0.0
        steer_angle = 0.0

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel = 0.0
                    steer_angle = 0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE

                if steer_angle > 1.0:
                    steer_angle = 1.0
                if steer_angle < -1.0:
                    steer_angle = -1.0

                print("Steer Angle", steer_angle)
                print("Linear Velocity", linear_vel)
                # Publish the twist message
                wheel_velocities.data = [
                    linear_vel, -linear_vel, linear_vel, -linear_vel]
                joint_positions.data = [steer_angle, steer_angle, 0.0, 0.0]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)"""

    def autonomous(self):
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        while True:
            # get the robot position
            # calculate the error value and pid output
            pose_error = self.robot_x - self.robot_y
            yaw_error = 1.0 - self.robot_yaw

            Kp_pose = 1.0
            Kp_yaw = 1.0
            pose_pid = pose_error * Kp_pose
            yaw_pid = yaw_error * Kp_yaw
            pid_output = pose_pid + yaw_pid

            # apply limits on pid output
            if (pid_output > 0.0):
                pid_output = 0.9
            elif (pid_output < -0.9):
                pid_output = -0.9
            
            # apply limits on velocities and steering angles
            steer_angle = pid_output
            linear_vel = 1.5

            if (self.robot_y > 10.0):
                steer_angle = 0.0
                linear_vel = 0.0

                self.get_logger().info("Robot reached the goal pose!")
            
            # publish the velocities and steering angle
            wheel_velocities.data = [
                linear_vel, -linear_vel, linear_vel, -linear_vel]
            joint_positions.data = [-steer_angle, -steer_angle, 0.0, 0.0]

            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.autonomous()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
