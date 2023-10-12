#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MoveForwardNode : public rclcpp::Node
{
public:
  MoveForwardNode() : Node("move_forward_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MoveForwardNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map"; // or whatever frame you're using
    msg.pose.position.x = 10.0;  // Move forward along x-axis
    msg.pose.position.y = 0.0;   // y-axis
    msg.pose.position.z = 10.0;   // z-axis
    msg.pose.orientation.w = 1.0; // Quaternion for no rotation
    RCLCPP_INFO(this->get_logger(), "Sending Pose: [%f, %f, %f]",
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveForwardNode>());
  rclcpp::shutdown();
  return 0;
}
