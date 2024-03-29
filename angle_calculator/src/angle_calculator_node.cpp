#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <random>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "angle_stamped_msg/msg/angle_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "drone_pose_stamped/msg/drone_pose_stamped.hpp"
#include "load_pose_stamped/msg/load_pose_stamped.hpp"

class AngleCalculator : public rclcpp::Node
{
public:
    AngleCalculator() : Node("angle_calculator")
    {
                auto qos = rclcpp::QoS(rclcpp::QoSInitialization(
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10
));
qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        load_pose_subscriber_ = this->create_subscription<load_pose_stamped::msg::LoadPoseStamped>(
            "/ros_load_pose", 10, std::bind(&AngleCalculator::on_load_pose_received, this, std::placeholders::_1));
        drone_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&AngleCalculator::on_drone_pose_received, this, std::placeholders::_1));
        load_angle_publisher_ = this->create_publisher<angle_stamped_msg::msg::AngleStamped>("load_angle", 10);
        clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10, std::bind(&AngleCalculator::on_clock_received, this, std::placeholders::_1));
    }

private:
    void on_load_pose_received(const load_pose_stamped::msg::LoadPoseStamped::SharedPtr msg)
    {
        if (msg == nullptr) {
            return;
        }
    // Specify standard deviations for IMU readings based on their range
    double pose_stddev = 0.0001;
    //msg->pose.position.x += generateWhiteNoise(pose_stddev);
    //msg->pose.position.y += generateWhiteNoise(pose_stddev);
    //msg->pose.position.z += generateWhiteNoise(pose_stddev);
    load_pose_ = msg;
    }

    void on_drone_pose_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg == nullptr) {
            return;
        }
    // Specify standard deviations for IMU readings based on their range
    double pose_stddev = 0.0001;
    //msg->pose.position.x += generateWhiteNoise(pose_stddev);
    //msg->pose.position.y += generateWhiteNoise(pose_stddev);
    //msg->pose.position.z += generateWhiteNoise(pose_stddev);
        drone_pose_ = msg; 
        calculate_angles();
    }

    void on_clock_received(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        latest_clock_ = msg->clock;
    }

void calculate_angles()
{
    if (!load_pose_ || !drone_pose_) {
        return;
    }

    tf2::Quaternion drone_orientation;
    tf2::convert(drone_pose_->pose.orientation, drone_orientation);
    drone_orientation.normalize();
    tf2::Matrix3x3 rotation_matrix(drone_orientation);
    tf2::Vector3 load_position(
        load_pose_->pose.position.x,
        load_pose_->pose.position.y,
        load_pose_->pose.position.z
    );

    tf2::Vector3 rotated_load_position = rotation_matrix * load_position;

    double x = rotated_load_position.x();
    double y = rotated_load_position.y();
    double z = rotated_load_position.z();
    RCLCPP_INFO(this->get_logger(), "x = %.2f y = %.2f z = %.2f",
            x, y, z);
    double projection_xy_magnitude = std::sqrt(x * x + y * y);

    double theta_x_rad = std::atan(x/(z));
    double theta_y_rad = std::atan(y/(z));
    double theta_z_rad = std::atan(projection_xy_magnitude/z);

    RCLCPP_INFO(this->get_logger(), "Theta: θ_x = %.2f rad, θ_y = %.2f rad, θ_z = %.2f rad",
                theta_x_rad, theta_y_rad, theta_z_rad);

    auto angle_msg = angle_stamped_msg::msg::AngleStamped();
    angle_msg.header.stamp = latest_clock_;
    angle_msg.angle.angle_x = theta_x_rad;
    angle_msg.angle.angle_y = theta_y_rad;
    angle_msg.angle.angle_z = theta_z_rad;
    load_angle_publisher_->publish(angle_msg);
}

double generateWhiteNoise(double stddev) {
    static std::random_device rd{};
    static std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, stddev};
    return d(gen);
}


    rclcpp::Subscription<load_pose_stamped::msg::LoadPoseStamped>::SharedPtr load_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    rclcpp::Publisher<angle_stamped_msg::msg::AngleStamped>::SharedPtr load_angle_publisher_;
    load_pose_stamped::msg::LoadPoseStamped::SharedPtr load_pose_;
    geometry_msgs::msg::PoseStamped::SharedPtr drone_pose_;
    builtin_interfaces::msg::Time latest_clock_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleCalculator>());
    rclcpp::shutdown();
    return 0;
}

