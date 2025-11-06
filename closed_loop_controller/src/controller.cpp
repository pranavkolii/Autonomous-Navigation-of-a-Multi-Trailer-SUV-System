#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
public:
  Controller() : Node("controller") {
    // declare ros2 parameters
    declare_parameter("position_command_topic",
                      "/position_controller/commands");
    declare_parameter("velocity_command_topic",
                      "/velocity_controller/commands");

    declare_parameter("robot_frame", "base_link");
    declare_parameter("map_frame", "map");

    // start the publishers
    {
      std::string vel_topic =
          get_parameter("velocity_command_topic").as_string();
      vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
          vel_topic, 10);

      std::string pos_topic =
          get_parameter("position_command_topic").as_string();
      pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
          pos_topic, 10);
    }

    // Initialize the TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // start timer callbacks for getting robot pose and controlling the robot
    update_robot_pose_timer_ = this->create_wall_timer(
        1s, std::bind(&Controller::update_robot_pose, this));

    std::this_thread::sleep_for(std::chrono::seconds(1));

    main_timer_ =
        this->create_wall_timer(1000ms, std::bind(&Controller::main_cb, this));

    RCLCPP_INFO(this->get_logger(), "Starting the closed loop control node, to "
                                    "move the SUV to (10.0, 10.0) point");
  }

private:
  // find TF from map to base_link frame, and write to robot_tf_ variable
  void update_robot_pose() {
    std::string robot_frame = get_parameter("robot_frame").as_string();
    std::string map_frame = get_parameter("map_frame").as_string();

    geometry_msgs::msg::TransformStamped transform_stamped;
    bool transform_available = true;

    try {
      // Look up the transform from the source frame to the target frame
      transform_stamped = tf_buffer_->lookupTransform(
          map_frame, robot_frame,
          tf2::TimePointZero); // Look up the latest available transform
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s",
                  map_frame.c_str(), robot_frame.c_str(), ex.what());
      transform_available = false;
    }

    if (transform_available) {
      tf_mutex_.lock();
      robot_pose_x_ = transform_stamped.transform.translation.x;
      robot_pose_y_ = transform_stamped.transform.translation.y;
      tf2::Quaternion quat(transform_stamped.transform.rotation.x,
                           transform_stamped.transform.rotation.y,
                           transform_stamped.transform.rotation.z,
                           transform_stamped.transform.rotation.w);
      tf2::Matrix3x3 m(quat); // Create a rotation matrix from the quaternion

      double roll, pitch;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_pose_yaw_);
      tf_mutex_.unlock();
    }
  }

  // main controlling thread
  void main_cb() {
    // get robot pose
    double robot_x, robot_y, robot_yaw;
    tf_mutex_.lock();
    robot_x = robot_pose_x_;
    robot_y = robot_pose_y_;
    robot_yaw = robot_pose_yaw_;
    tf_mutex_.unlock();

    // calculate pose and yaw error for robot
    double pose_error = robot_x - robot_y;
    double yaw_error = 1.0 - robot_pose_yaw_;

    // compute PID output for the error
    double Kp_pose = 1.0;
    double Kp_yaw = 1.0;
    double pose_pid = pose_error * Kp_pose;
    double yaw_pid = yaw_error * Kp_yaw;
    double pid_output = pose_pid + yaw_pid;

    // apply limits on pid output
    if (pid_output > 0.9) {
      pid_output = 0.9;
    } else if (pid_output < -0.9) {
      pid_output = -0.9;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Current robot pose(x, y, yaw) (%.2lf, %.2lf, %.2lf)\n"
        "Controller (pose_error, yaw_error, pid_output) (%.2lf, %.2lf, %.2lf)",
        robot_x, robot_y, robot_yaw, pose_error, yaw_error, pid_output);

    // publish velocity and steering angles
    double steer_angle = pid_output;
    double linear_vel = 1.5;
    if (robot_y > 10.0) // goal reached, stop the robot
    {
      steer_angle = 0.0;
      linear_vel = 0.0;

      RCLCPP_INFO(this->get_logger(), "Goal pose reached by the SUV!");
    }

    auto vel_msg = std_msgs::msg::Float64MultiArray();
    vel_msg.data = {linear_vel, linear_vel, linear_vel, linear_vel,
                    linear_vel-0.1, linear_vel-0.1, linear_vel-0.1, linear_vel-0.1};
    vel_pub_->publish(vel_msg);

    auto pos_msg = std_msgs::msg::Float64MultiArray();
    pos_msg.data = {steer_angle, steer_angle};
    pos_pub_->publish(pos_msg);
  }

  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;

  std::vector<geometry_msgs::msg::Pose> target_poses_;

  rclcpp::TimerBase::SharedPtr update_robot_pose_timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  double robot_pose_x_, robot_pose_y_, robot_pose_yaw_;
  std::mutex tf_mutex_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
