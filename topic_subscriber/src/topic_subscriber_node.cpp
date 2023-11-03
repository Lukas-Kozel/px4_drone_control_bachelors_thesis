#include "rclcpp/rclcpp.hpp"
#include "load_pose_stamped/msg/load_pose_stamped.hpp"  // Include your custom message header


class LoadPoseSubscriber : public rclcpp::Node
{
public:
  LoadPoseSubscriber()
  : Node("load_pose_subscriber")
  {
    subscription_ = this->create_subscription<my_custom_msgs::msg::LoadPoseStamped>(
      "/load_pose", 10, std::bind(&LoadPoseSubscriber::callback, this, std::placeholders::_1));
  }

private:
  void callback(const my_custom_msgs::msg::LoadPoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message");
    RCLCPP_INFO(this->get_logger(), "Timestamp (sec): %ld, (nsec): %d", msg->header.stamp.sec, msg->header.stamp.nsec);
    
    // Assuming you have a KeyValue message or similar for the data map
    for (size_t i = 0; i < msg->header.keys.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "%s: %s", msg->header.keys[i].c_str(), msg->header.values[i].c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "Position (X, Y, Z): (%f, %f, %f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Orientation (W, X, Y, Z): (%f, %f, %f, %f)", msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  }

  rclcpp::Subscription<my_custom_msgs::msg::LoadPoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoadPoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}