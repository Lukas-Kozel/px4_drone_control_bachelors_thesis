#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/vector3.hpp"

class AngleCalculator : public rclcpp::Node
{
public:
    AngleCalculator() : Node("angle_calculator")
    {
        load_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/load_pose", 10, std::bind(&AngleCalculator::on_load_pose_received, this, std::placeholders::_1));
        drone_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/drone_pose", 10, std::bind(&AngleCalculator::on_drone_pose_received, this, std::placeholders::_1));
        load_angle_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("load_angle", 10);
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


        auto angle_msg = geometry_msgs::msg::Vector3();
        angle_msg.x = theta_x_rad;
        angle_msg.y = theta_y_rad;
        angle_msg.z = theta_z_rad;
        load_angle_publisher_->publish(angle_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr load_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr drone_pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr load_angle_publisher_;
    geometry_msgs::msg::Pose::SharedPtr load_pose_;
    geometry_msgs::msg::Pose::SharedPtr drone_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleCalculator>());
    rclcpp::shutdown();
    return 0;
}
