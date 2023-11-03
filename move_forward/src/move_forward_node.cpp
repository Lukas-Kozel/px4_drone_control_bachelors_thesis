#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"

class MoveForwardNode : public rclcpp::Node
{
public:
  MoveForwardNode() : Node("move_forward_node")
  {
    auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10  // The depth of the QoS history, similar to your original code
));
qos2.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
    drone_state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos2, std::bind(&MoveForwardNode::on_drone_state_received, this, std::placeholders::_1));
    
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

   void on_drone_state_received(const mavros_msgs::msg::State::SharedPtr msg)
    {
        if (msg == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Received null message in on_drone_state_received");
            return;
        }
        bool offboard_mode_ = (msg->mode== "OFFBOARD");
        RCLCPP_INFO(this->get_logger(), "it is working");
        
    }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
      rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr drone_state_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveForwardNode>());
  rclcpp::shutdown();
  return 0;
}
