#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Custom clamp function
double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

class PIDControllerNode : public rclcpp::Node
{
public:
  PIDControllerNode() : Node("pid_controller_node")
  {
    RCLCPP_INFO(this->get_logger(), "Node is being constructed and subscription is being created");
    auto qos_profile = rclcpp::SystemDefaultsQoS();
    // Reduced PID gains for less aggressive control
    Kp = 1.05;
    Ki = 0.05;
    Kd = 0.1;

    // Initialize error and integral terms for all axes
    prev_error_x = prev_error_y = 0.0;
    integral_x = integral_y =  0.0;

    // Integral wind-up guard
    windup_guard = 25.0;

    is_initialized = false;
    // Subscribe to IMU data
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/load_imu", qos_profile, std::bind(&PIDControllerNode::imu_callback, this, std::placeholders::_1));

    // Publish control commands
    control_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", qos_profile);

    // Acceptable error range (tolerance)
    error_tolerance = 0.01;  // Adjust this value as needed
    // TODO: Add logic to check if the drone is in Offboard mode

    //get z axis of drone:
    altitude_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", qos_profile, std::bind(&PIDControllerNode::altitude_callback, this, std::placeholders::_1));


  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // TODO: Check if the drone is in Offboard mode before proceeding

    if (!is_initialized) {
        RCLCPP_WARN(this->get_logger(), "Waiting for initial altitude...");
        return;
    }
    // Extract relevant IMU data for all axes
    double angular_velocity_x = msg->angular_velocity.x;
    double angular_velocity_y = msg->angular_velocity.y;
    double angular_velocity_z = msg->angular_velocity.z;

    // Compute error for each axis
    double error_x = clamp(0 - angular_velocity_x, -error_tolerance, error_tolerance);
    double error_y = clamp(0 - angular_velocity_y, -error_tolerance, error_tolerance);
    double error_z = clamp(0 - angular_velocity_z, -error_tolerance, error_tolerance);

    // Compute integral and apply wind-up guard
    integral_x = clamp(integral_x + error_x, -windup_guard, windup_guard);
    integral_y = clamp(integral_y + error_y, -windup_guard, windup_guard);
    integral_z = clamp(integral_z + error_z, -windup_guard, windup_guard);
    // Compute derivative
    double derivative_x = error_x - prev_error_x;
    double derivative_y = error_y - prev_error_y;
    double derivative_z = error_z - prev_error_z;

    RCLCPP_INFO(this->get_logger(), "Sending Pose: [%f, %f, %f]",
    integral_x, integral_y, integral_z);

    // Compute control command using PID equation for each axis
    double control_command_x = Kp * error_x + Ki * integral_x + Kd * derivative_x;
    double control_command_y = Kp * error_y + Ki * integral_y + Kd * derivative_y;
    double control_command_z = Kp * error_z + Ki * integral_z + Kd * derivative_z;

    // Create and populate PoseStamped message
    auto control_msg = geometry_msgs::msg::PoseStamped();
    control_msg.header.stamp = this->now();
    control_msg.header.frame_id = "map";
    control_msg.pose.position.x = control_command_x;
    control_msg.pose.position.y = control_command_y;
    control_msg.pose.position.z = control_command_z;

    // Publish control command
    control_publisher_->publish(control_msg);

    // Update previous error for each axis
    prev_error_x = error_x;
    prev_error_y = error_y;
    prev_error_z = error_z;
  }
  void altitude_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!is_initialized) {
        double initial_altitude = msg->pose.position.z;  // Get the z-coordinate (altitude)
        prev_error_z = initial_altitude;  // Initialize prev_error_z
        integral_z = initial_altitude;    // Initialize integral_z
        is_initialized = true;  // Mark as initialized
    }
  }


  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr control_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr altitude_subscription_;
  double Kp, Ki, Kd;
  double prev_error_x, prev_error_y, prev_error_z, integral_x, integral_y, integral_z;
  double windup_guard;  // Integral wind-up guard
  double error_tolerance;
  bool is_initialized;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}
