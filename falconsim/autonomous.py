import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import math

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')

        # Publishers to match your TELEOP topics in suv.py
        self.pos_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Subscribe to the pose topic published by suv.py twin
        self.pose_sub = self.create_subscription(Pose, '/autonomous_controller/pose', self.pose_callback, 10)

        self.goal_x = 10.0
        self.goal_y = 10.0
        self.kp_steering = 1.0
        self.max_steering_angle = math.pi / 4  # 45 degrees max steering
        self.max_velocity = 1.0  # Max throttle normalized

    def pose_callback(self, msg: Pose):
        # Calculate error to goal
        error_x = self.goal_x - msg.position.x
        error_y = self.goal_y - msg.position.y

        dist_to_goal = math.sqrt(error_x**2 + error_y**2)

        # Stop near goal
        if dist_to_goal < 0.5:
            linear_vel = 0.0
            steering = 0.0
            self.get_logger().info('Goal reached, stopping vehicle.')
        else:
            # Control law: steering angle proportional to angle to goal, clamped [-1,1]
            angle_to_goal = math.atan2(error_y, error_x)
            steering = self.kp_steering * angle_to_goal / self.max_steering_angle
            steering = max(min(steering, 1.0), -1.0)

            # Throttle proportional to distance, max 1.0
            linear_vel = min(dist_to_goal, self.max_velocity)

        # Publish velocity command - list size will depend on your twin, here 8 wheels example
        vel_msg = Float64MultiArray()
        vel_msg.data = [linear_vel] * 4
        self.vel_pub.publish(vel_msg)

        # Publish steering command - commonly 2 steering joints
        pos_msg = Float64MultiArray()
        pos_msg.data = [steering, steering]
        self.pos_pub.publish(pos_msg)

        self.get_logger().info(f'Pos ({msg.position.x:.2f},{msg.position.y:.2f}), '
                               f'Dist {dist_to_goal:.2f}, Steering {steering:.2f}, Velocity {linear_vel:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

