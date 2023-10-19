#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "state_vector_msg/msg/state_vector.hpp"
#include "load_pose_stamped/msg/load_pose_stamped.hpp"
#include "drone_pose_stamped/msg/drone_pose_stamped.hpp"
#include "angle_stamped_msg/msg/angle_stamped.hpp"
class StateSyncNode : public rclcpp::Node
{
public:
  typedef message_filters::sync_policies::ApproximateTime<drone_pose_stamped::msg::DronePoseStamped, sensor_msgs::msg::Imu, sensor_msgs::msg::Imu> MySyncPolicy; //load_pose_stamped::msg::LoadPoseStamped, drone_pose_stamped::msg::DronePoseStamped, sensor_msgs::msg::Imu, angle_stamped_msg::msg::AngleStamped, geometry_msgs::msg::TwistStamped, sensor_msgs::msg::Imu
  StateSyncNode() : Node("state_sync_node")
  {
//   auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(10));
 //  custom_qos.best_effort();
  //  auto rmw_qos = custom_qos.get_rmw_qos_profile();
    // Initialize subscribers
    //load_pose_sub_ = std::make_shared<message_filters::Subscriber<load_pose_stamped::msg::LoadPoseStamped>>(this, "/ros_load_pose");
    drone_pose_sub_ = std::make_shared<message_filters::Subscriber<drone_pose_stamped::msg::DronePoseStamped>>(this, "/ros_drone_pose");
    load_imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, "/load_imu");
    //load_angle_sub_ = std::make_shared<message_filters::Subscriber<angle_stamped_msg::msg::AngleStamped>>(this, "/load_angle", rmw_qos);
    //drone_velocity_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(this, "/mavros/local_position/velocity_local", rmw_qos);
    drone_imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, "/drone_imu");


    // Initialize synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10),*load_pose_sub_, *load_imu_sub_, *drone_imu_sub_); // *load_pose_sub_, *drone_pose_sub_, *load_imu_sub_, *load_angle_sub_, *drone_velocity_sub_, *drone_imu_sub_
    sync_->registerCallback(&StateSyncNode::callback, this);

    // Initialize publisher
    pub_ = this->create_publisher<state_vector_msg::msg::StateVector>("state_vector", 10);
    RCLCPP_INFO(this->get_logger(), "Successfully started");
  }

private:
  void callback(const load_pose_stamped::msg::LoadPoseStamped::SharedPtr& load_pose,
                //const drone_pose_stamped::msg::DronePoseStamped::SharedPtr& drone_pose,
                const sensor_msgs::msg::Imu::SharedPtr& load_imu,
                //const angle_stamped_msg::msg::AngleStamped::SharedPtr& load_angle,
                //const geometry_msgs::msg::TwistStamped::SharedPtr& drone_velocity,
                const sensor_msgs::msg::Imu::SharedPtr& drone_imu)
  {

    // Publish the synchronized data
    state_vector_msg::msg::StateVector msg;
    msg.load_pose = *load_pose;
    //msg.drone_pose = *drone_pose;
    msg.load_imu = *load_imu;
    //msg.load_angle = *load_angle;
    //msg.drone_velocity.twist = drone_velocity->twist;
    msg.drone_imu = *drone_imu;
    // Fill in the msg fields
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), 
      "Publishing data: load_imu: x=%.2f y=%.2f, drone_velocity: x=%.2f y=%.2f,  drone_imu: x=%.2f y=%.2f", //load_pose: x=%.2f y=%.2f, drone_pose: x=%.2f y=%.2f, load_imu: x=%.2f y=%.2f, load_angle: x=%.2f y=%.2f, drone_velocity: x=%.2f y=%.2f, drone_imu: x=%.2f y=%.2f",
      load_pose->pose.position.x, load_pose->pose.position.y,
      //drone_pose->pose.position.x, drone_pose->pose.position.y,
      load_imu->linear_acceleration.x, load_imu->linear_acceleration.y,
      //load_angle->angle.angle_x, load_angle->angle.angle_y,
      // drone_velocity->twist.linear.x, drone_velocity->twist.linear.y,
      drone_imu->linear_acceleration.x, drone_imu->linear_acceleration.y
    );
  }

    std::shared_ptr<message_filters::Subscriber<load_pose_stamped::msg::LoadPoseStamped>> load_pose_sub_;
    //std::shared_ptr<message_filters::Subscriber<drone_pose_stamped::msg::DronePoseStamped>> drone_pose_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> load_imu_sub_;
    //std::shared_ptr<message_filters::Subscriber<angle_stamped_msg::msg::AngleStamped>> load_angle_sub_;
    //std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> drone_velocity_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> drone_imu_sub_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    rclcpp::Publisher<state_vector_msg::msg::StateVector>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}