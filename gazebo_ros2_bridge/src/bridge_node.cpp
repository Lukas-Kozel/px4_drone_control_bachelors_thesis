#include "rclcpp/rclcpp.hpp"
#include <gz/transport/Node.hh>
#include "load_pose_stamped.pb.h"
#include "load_pose_stamped/msg/load_pose_stamped.hpp"

class GazeboRosBridge : public rclcpp::Node
{
public:
  GazeboRosBridge()
  : Node("gazebo_ros_bridge")
  {
    publisher_load_ = this->create_publisher<load_pose_stamped::msg::LoadPoseStamped>("/ros_load_pose", 10);
    subscriber_load_ = std::make_unique<gz::transport::v12::Node>();
    subscriber_load_->Subscribe("/load_pose", &GazeboRosBridge::OnLoadMsg, this);
  }

private:
  void OnLoadMsg(const gz_pose_plugin::LoadPoseStamped &gz_msg)
  {
    auto ros_msg = load_pose_stamped::msg::LoadPoseStamped();

    ros_msg.header.stamp.sec = gz_msg.header().stamp().sec();
    ros_msg.header.stamp.nanosec = gz_msg.header().stamp().nsec() *1e3;
    
    ros_msg.pose.position.x = gz_msg.pose().position().x();
    ros_msg.pose.position.y = gz_msg.pose().position().y();
    ros_msg.pose.position.z = gz_msg.pose().position().z();

    ros_msg.pose.orientation.w = gz_msg.pose().orientation().w();
    ros_msg.pose.orientation.x = gz_msg.pose().orientation().x();
    ros_msg.pose.orientation.y = gz_msg.pose().orientation().y();
    ros_msg.pose.orientation.z = gz_msg.pose().orientation().z();

    RCLCPP_INFO(this->get_logger(), "Received LoadPoseStamped message: position(%f, %f, %f), orientation(%f, %f, %f, %f)",
                gz_msg.pose().position().x(), gz_msg.pose().position().y(), gz_msg.pose().position().z(),
                gz_msg.pose().orientation().w(), gz_msg.pose().orientation().x(), gz_msg.pose().orientation().y(), gz_msg.pose().orientation().z());

    publisher_load_->publish(ros_msg);
  }

  rclcpp::Publisher<load_pose_stamped::msg::LoadPoseStamped>::SharedPtr publisher_load_;
  std::unique_ptr<gz::transport::v12::Node> subscriber_load_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeboRosBridge>();
  rclcpp::spin(node);
  return 0;
}
