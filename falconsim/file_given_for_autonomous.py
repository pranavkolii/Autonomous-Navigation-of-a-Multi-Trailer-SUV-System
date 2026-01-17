import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import time

gx, gy = 10, 10

class AutonomyNode(Node):

   def __init__(self):
       super().__init__('autonomy_node')

       self.joint_position_pub = self.create_publisher(
           Float64MultiArray, '/position_controller/commands', 10)
       self.wheel_velocities_pub = self.create_publisher(
           Float64MultiArray, '/velocity_controller/commands', 10)
       self.pose_sub = self.create_subscription(
           Pose, '/autonomous_controller/pose', self.pose_callback, 10)

       self.start_time = time.time()
       self.times = [] # keeping track of times
       self.xs = [] # keeping track of x positions
       self.ys = [] # keeping track of y positions

   def pose_callback(self, msg):
       # TODO: Implement a proportional controller to move towards (gx, gy)
       # u is the control output

       # wheel_velocities = Float64MultiArray()
       # wheel_velocities.data = [u, -u, u, -u]
       # self.wheel_velocities_pub.publish(wheel_velocities)

       # print("Control Output (u):", u)
       # print("Current Position:", msg.position.x, msg.position.y)

       # TODO: Add values to times, xs, ys lists
       pass

def main(args=None):
   rclpy.init(args=args)
   node = AutonomyNode()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()

       # TODO: Plot figure of trajectory

       rclpy.shutdown()


if __name__ == '__main__':
   main()
