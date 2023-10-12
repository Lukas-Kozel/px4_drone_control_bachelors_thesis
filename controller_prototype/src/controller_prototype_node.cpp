#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "pid.h"

class PIDControllerNode : public rclcpp::Node
{
public:
  PIDControllerNode() : Node("controller_prototype_node")
  {
    RCLCPP_INFO(this->get_logger(), "Node is being constructed and subscription is being created");
    auto qos_profile = rclcpp::SystemDefaultsQoS();
    // Initialize the PID controller
    pid_init(&pid, PID_MODE_DERIVATIV_CALC, 0.01);
    pid_set_parameters(&pid, 1.05, 0.15, 0.05, 15, 15);

    is_initialized = false;

    // Subscribe to IMU data
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/load_imu", qos_profile, std::bind(&PIDControllerNode::imu_callback, this, std::placeholders::_1));

    // Publish control commands
    control_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", qos_profile);

    // Subscribe to altitude data
    altitude_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", qos_profile, std::bind(&PIDControllerNode::altitude_callback, this, std::placeholders::_1));
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!is_initialized) {
        RCLCPP_WARN(this->get_logger(), "Waiting for initial altitude...");
        return;
    }

    // Use the new PID methods to calculate control commands
    double control_command_x = pid_calculate(&pid, 0, msg->angular_velocity.x, 0, 0.01);
    double control_command_y = pid_calculate(&pid, 0, msg->angular_velocity.y, 0, 0.01);
    double control_command_z = pid_calculate(&pid, 0, msg->angular_velocity.z, 0, 0.01);

    // Create and populate PoseStamped message
    auto control_msg = geometry_msgs::msg::PoseStamped();
    control_msg.header.stamp = this->now();
    control_msg.header.frame_id = "map";
    control_msg.pose.position.x = control_command_x;
    control_msg.pose.position.y = control_command_y;
    control_msg.pose.position.z = control_command_z;

    // Publish control command
    control_publisher_->publish(control_msg);
    RCLCPP_INFO(this->get_logger(), "Sending Pose: [%f, %f, %f]",
    control_command_x, control_command_y, control_command_z);
  }

  void altitude_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!is_initialized) {
        pid.integral = msg->pose.position.z;  // Initialize integral
        is_initialized = true;  // Mark as initialized
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr control_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr altitude_subscription_;
  bool is_initialized;
  PID_t pid;  // New PID structure
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}
