#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"
#include "sensor_msgs/msg/imu.hpp"

class LQRController : public rclcpp::Node
{
public:
    LQRController() : Node("lqr_controller"), control_timer_(nullptr)
    {
        load_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/load_pose", 10, std::bind(&LQRController::on_load_pose_received, this, std::placeholders::_1));
        drone_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/drone_pose", 10, std::bind(&LQRController::on_drone_pose_received, this, std::placeholders::_1));
        load_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/load_imu", 10, std::bind(&LQRController::on_load_imu_received, this, std::placeholders::_1));
        load_angle_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/load_angle", 10, std::bind(&LQRController::on_load_angle_received, this, std::placeholders::_1));
        drone_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/mavros/local_position/velocity_local", 10, std::bind(&LQRController::on_drone_velocity_received, this, std::placeholders::_1));
        drone_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/drone_imu", 10, std::bind(&LQRController::on_drone_imu_received, this, std::placeholders::_1));
        
        state_x = Eigen::VectorXd::Zero(6);
        state_y = Eigen::VectorXd::Zero(6);
        K_x_ = (Eigen::MatrixXd(6, 1) << 1, 2, 3, 4, 5, 6).finished();
        K_y_ = (Eigen::MatrixXd(6, 1) << 1, 2, 3, 4, 5, 6).finished();
        previous_angular_velocity_x_ = 0;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Adjust the time interval to your needs
            std::bind(&LQRController::control, this)
        );
    }

private:
    void on_load_pose_received(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        load_pose_ = msg;
    }
    void on_drone_pose_received(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        drone_pose_ = msg;
    }
    void on_load_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        load_imu_ = msg;
    }
    void on_load_angle_received(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        load_angle_ = msg;
    }
    void on_drone_velocity_received(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        drone_velocity_ = msg;
    }
    void on_drone_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        drone_imu_ = msg;
    }

    void create_state_vector(){
        state_x(0)= load_pose_->position.x + drone_pose_->position.x; //position
        state_x(1)= drone_velocity_->linear.x;       //velocity
        state_x(2)= drone_imu_ ->linear_acceleration.x;
        state_x(3)= load_angle_-> x;
        state_x(4)= load_imu_->angular_velocity.x;
        state_x(5)= load_imu_->linear_acceleration.x;

        state_y(0)= load_pose_->position.y + drone_pose_->position.y; //position
        state_y(1)= drone_velocity_->linear.y;       //velocity
        state_y(2)= drone_imu_ ->linear_acceleration.y;
        state_y(3)= load_angle_-> y;
        state_y(4)= load_imu_->angular_velocity.y;
        state_y(5)= load_imu_->linear_acceleration.y;
        
    }

    void control(){
        create_state_vector();
        control_input_x = -K_x_ * state_x;
        control_input_y = -K_y_ * state_y;
        //here add publishing to drone

    }
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr load_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr load_imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr load_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drone_velocity_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr drone_imu_subscriber_;
    geometry_msgs::msg::Pose::SharedPtr load_pose_;
    geometry_msgs::msg::Pose::SharedPtr drone_pose_;
    geometry_msgs::msg::Vector3::SharedPtr load_angle_;
    sensor_msgs::msg::Imu::SharedPtr load_imu_;
    sensor_msgs::msg::Imu::SharedPtr drone_imu_;
    geometry_msgs::msg::Twist::SharedPtr drone_velocity_;
    Eigen::MatrixXd K_x_;
    Eigen::MatrixXd K_y_;
    Eigen::VectorXd state_x;
    Eigen::VectorXd state_y;
    Eigen::VectorXd control_input_x;
    Eigen::VectorXd control_input_y;
    double previous_angular_velocity_x_;  // add this line as a class member variable

};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRController>());
    rclcpp::shutdown();
    return 0;
}
