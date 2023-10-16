#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "angle_stamped_msg/msg/angle_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class AngleCalculator : public rclcpp::Node
{
public:
    AngleCalculator() : Node("angle_calculator")
    {
        load_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/load_pose", 10, std::bind(&AngleCalculator::on_load_pose_received, this, std::placeholders::_1));
        drone_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/drone_pose", 10, std::bind(&AngleCalculator::on_drone_pose_received, this, std::placeholders::_1));
        load_angle_publisher_ = this->create_publisher<angle_stamped_msg::msg::AngleStamped>("load_angle", 10);
        clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10, std::bind(&AngleCalculator::on_clock_received, this, std::placeholders::_1));
    }

private:
    void on_load_pose_received(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        load_pose_ = msg;
        calculate_angles();
    }

    void on_drone_pose_received(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        drone_pose_ = msg;
        calculate_angles();
    }

    void on_clock_received(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        latest_clock_ = msg->clock;
    }
        angle_stamped_msg::msg::AngleStamp convert_to_angle_stamp(const builtin_interfaces::msg::Time& time)
    {
        angle_stamped_msg::msg::AngleStamp angle_stamp;
        angle_stamp.sec = time.sec;
        angle_stamp.nsec = time.nanosec;
        return angle_stamp;
    }
    void calculate_angles()
    {
        if (!load_pose_ || !drone_pose_) {
            return;
        }

        double x = load_pose_->position.x;
        double y = load_pose_->position.y;
        double z = load_pose_->position.z;

        tf2::Quaternion q(
            drone_pose_->orientation.x,
            drone_pose_->orientation.y,
            drone_pose_->orientation.z,
            drone_pose_->orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double adjusted_x = x * cos(pitch) + z * sin(pitch);
        double adjusted_y = y * cos(roll) - z * sin(roll);
        double adjusted_z = -x * sin(pitch) + z * cos(pitch);
        double projection_xy_magnitude = std::sqrt(adjusted_x * adjusted_x + adjusted_y * adjusted_y);

        if(projection_xy_magnitude == 0) {
            RCLCPP_WARN(this->get_logger(), "Projection onto xy-plane has zero magnitude, cannot calculate θ_z");
            return;
        }

        double theta_x_rad = std::atan2(adjusted_z, adjusted_x);
        double theta_y_rad = std::atan2(adjusted_z, adjusted_y);
        double theta_z_rad = std::atan2(projection_xy_magnitude, -adjusted_z);

        theta_x_rad = theta_x_rad + M_PI/2;
        theta_y_rad = theta_y_rad + M_PI/2;
        theta_z_rad = theta_z_rad;

        RCLCPP_INFO(this->get_logger(), "Theta: θ_x = %.2f rad, θ_y = %.2f rad, θ_z = %.2f rad",
                    theta_x_rad, theta_y_rad, theta_z_rad);

        auto angle_msg = angle_stamped_msg::msg::AngleStamped();
        angle_msg.header.stamp = convert_to_angle_stamp(latest_clock_);
        angle_msg.angle.angle_x = theta_x_rad;
        angle_msg.angle.angle_y = theta_y_rad;
        angle_msg.angle.angle_z = theta_z_rad;
        load_angle_publisher_->publish(angle_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr load_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    rclcpp::Publisher<angle_stamped_msg::msg::AngleStamped>::SharedPtr load_angle_publisher_;
    geometry_msgs::msg::Pose::SharedPtr load_pose_;
    geometry_msgs::msg::Pose::SharedPtr drone_pose_;
    builtin_interfaces::msg::Time latest_clock_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleCalculator>());
    rclcpp::shutdown();
    return 0;
}
