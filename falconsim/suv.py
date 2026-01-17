from peregrine.twins import SystemTwin
import os
import dupy_unreal as dupy
from peregrine.conf import global_settings
global_settings.Settings.BASE_OUTPUT_DIRECTORY = os.path.dirname(dupy.find_object(name=u'DuRTLFunctionLibrary', _class=dupy.find_object(name=u'Class')).get_cdo().call_function("GetCurrentScenarioFilename")) + "/Output"

from peregrine.pipelines.ros2.utils import ros2_env_setup
ros2_env_setup()
import rclpy
if not rclpy.ok():
  rclpy.init()
from peregrine.twins import SystemTwin
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray

CM_TO_M = 0.01

# TODO: set the correct topic names expected by your teleop stack
TELEOP_STEER_TOPIC = '/position_controller/commands'       # TODO: verify/change
TELEOP_VEL_TOPIC   = '/velocity_controller/commands'       # TODO: verify/change


# TODO: set the output pose topic
POSE_TOPIC         = '/autonomous_controller/pose'         # TODO: verify/change

def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

class suv(SystemTwin):
    def begin_play(self):
        super().begin_play()
        self.teleop_subscriber = TeleopSubscriber()
        self.sensor_manager.start()
        self.vehicle_movement_component = self.get_property("VehicleMovementComponent")
        

    def tick(self, delta_time):
        super().tick(delta_time)
        self.sensor_manager.capture_and_run_pipelines(self.sim_time)
        rclpy.spin_once(self.teleop_subscriber, timeout_sec=0.01)
        vel_cmd  = self.teleop_subscriber.input_vel
        turn_cmd = self.teleop_subscriber.input_turn
        throttle = clamp(vel_cmd, -1.0, 1.0)
        steering = clamp(turn_cmd, -1.0, 1.0)
        self.vehicle_movement_component.call_function("SetThrottleInput", throttle)
        self.vehicle_movement_component.call_function("SetSteeringInput", steering)
        # === POSE PUBLISH ===
        # 1) Get actor transform
        transform = self.get_actor_transform()
        # 2) Extract and convert position from cm to meters
        position = transform['Translation']
        pose = Pose()
        pose.position.x = position['X'] * CM_TO_M
        pose.position.y = position['Y'] * CM_TO_M
        pose.position.z = position['Z'] * CM_TO_M

        # 3) Optionally set orientation (or identity)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        # 4) Publish using the TeleopSubscriber's helper
        self.teleop_subscriber.publish_pose(self)

    def end_play(self):
        super().end_play()
        self.sensor_manager.stop()

class TeleopSubscriber(Node):
    def __init__(self):
        super().__init__('teleop_subscriber')

        # Internal state the truck reads each tick
        self.input_vel: float = 0.0   # normalized forward command, expected ~[-1, 1]
        self.input_turn: float = 0.0  # normalized steering command, expected ~[-1, 1]

        # --- Subscriptions ---
        self.steer_sub = self.create_subscription(
            Float64MultiArray,
            TELEOP_STEER_TOPIC,
            self.steer_callback,
            10
        )
        self.vel_sub = self.create_subscription(
            Float64MultiArray,
            TELEOP_VEL_TOPIC,
            self.vel_callback,
            10
        )
        _ = self.steer_sub, self.vel_sub  # avoid lints

        # --- Publisher ---
        self.pose_pub = self.create_publisher(Pose, POSE_TOPIC, 10)

    # ======== STUDENT CALLBACKS ========
    def steer_callback(self, msg: Float64MultiArray):
        """
        Extract the steering command from msg.data[0].
        Normalize/clamp to [-1, 1].
        Invert sign if necessary to match vehicle steering direction.
        """
        raw = msg.data[0]
        # Assuming raw steering command is in degrees or normalized scale,
        # here we assume it is already in range [-1,1], or scale accordingly.
        # For example, if raw is in degrees [-30,30], normalize:
        # steering = clamp(raw / 30.0, -1.0, 1.0)
        # For general case, just clamp raw here:
        steering = max(min(raw, 1.0), -1.0)
        self.input_turn = steering

    def vel_callback(self, msg: Float64MultiArray):
        """
        Extract velocity command from msg.data[0].
        Normalize/clamp to [-1, 1].
        """
        raw = msg.data[0]
        # Typically, velocity commands might be in some range, e.g., [-100, 100]
        # Normalize assuming max magnitude 100; adjust as needed for your system
        velocity = max(min(raw / 100.0, 1.0), -1.0)
        self.input_vel = velocity

    # ======== STUDENT POSE PUBLISH ========
    def publish_pose(self, twin):
        """
        Query actor transform from the twin.
        Convert position from cm to meters.
        Fill Pose message with position.
        Use identity quaternion orientation.
        Publish pose.
        """
        transform = twin.get_actor_transform()
        pos_cm = transform['Translation']

        pose = Pose()
        pose.position.x = pos_cm['X'] * CM_TO_M + 131.39
        pose.position.y = pos_cm['Y'] * CM_TO_M + 65.51
        pose.position.z = pos_cm['Z'] * CM_TO_M

        # Orientation: identity quaternion (no rotation)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

