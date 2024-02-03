#include "rclcpp/rclcpp.hpp"
#include "PIDController.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "drone_pose_stamped/msg/drone_pose_stamped.hpp"
#include "load_pose_stamped/msg/load_pose_stamped.hpp"
#include "angle_stamped_msg/msg/angle_stamped.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"



class LQRController : public rclcpp::Node
{
public:
    LQRController() : Node("lqr_controller"), control_timer_(nullptr)
    {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10
));
qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        drone_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&LQRController::on_drone_pose_received, this, std::placeholders::_1));
        load_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/load_imu", 10, std::bind(&LQRController::on_load_imu_received, this, std::placeholders::_1));
        load_angle_subscriber_ = this->create_subscription<angle_stamped_msg::msg::AngleStamped>(
            "/load_angle", 10, std::bind(&LQRController::on_load_angle_received, this, std::placeholders::_1));
        drone_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", qos, std::bind(&LQRController::on_drone_velocity_received, this, std::placeholders::_1));
        K_matrix_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/dlq_k_matrix", 10, std::bind(&LQRController::on_K_matrix_received, this, std::placeholders::_1));
        attitude_publisher_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 20);
        state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("state_vector", 10);
        state_x = Eigen::VectorXd::Zero(4);
        state_y = Eigen::VectorXd::Zero(4);
        loadLQRParams();
        system_mass = 2.25;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  
            std::bind(&LQRController::control, this)
        );
    }

private:

    void on_K_matrix_received(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        if(msg == nullptr){
            RCLCPP_INFO(this->get_logger(),"not updated");
            return;
        }
        Eigen::VectorXd K(msg->data.size());
        for(size_t i = 0; i < msg->data.size(); ++i) {
        K(i) = msg->data[i];
        RCLCPP_INFO(this->get_logger(),"updated");
    }
    }

    void on_drone_pose_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    drone_pose_ = msg;
}
    void on_load_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
    {        
    if (msg == nullptr) {
        return;
    }
    load_imu_ = msg;   
    }
    void on_load_angle_received(const angle_stamped_msg::msg::AngleStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    load_angle_ = msg;
}
    void on_drone_velocity_received(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    drone_velocity_ = msg;
}

void update_load_angular_velocity() {
    if (!load_imu_ || !drone_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Load IMU or drone pose data not available");
        return;
    }

    tf2::Quaternion load_orientation(
        load_imu_->orientation.x,
        load_imu_->orientation.y,
        load_imu_->orientation.z,
        load_imu_->orientation.w
    );
    load_orientation.normalize();
    double load_roll, load_pitch, load_yaw;
    tf2::Matrix3x3(load_orientation).getRPY(load_roll, load_pitch, load_yaw);
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setRPY(0, 0, load_yaw);
        
    // Obtain angular velocity from IMU data
    tf2::Vector3 local_angular_velocity(
        load_imu_->angular_velocity.x,
        load_imu_->angular_velocity.y,
        load_imu_->angular_velocity.z
    );
    RCLCPP_INFO(this->get_logger(), "yaw: %.2f",load_yaw);
    tf2::Vector3 rotated_load_imu = rotation_matrix.transpose() * local_angular_velocity;
    state_x(3) = rotated_load_imu.x();
    state_y(3) = rotated_load_imu.y();
}

    void create_state_vector(){
        if (!drone_pose_ || !load_imu_ || !load_angle_ || !drone_velocity_) {
        RCLCPP_ERROR(this->get_logger(), "Missing required data for state vector creation");
        return;
    }
        state_x(0) = drone_pose_->pose.position.x;
        state_x(1) = drone_velocity_->twist.linear.x;
        state_x(2)= load_angle_-> angle.angle_x;
        //state_x(3)= load_imu_->angular_velocity.x;


        state_y(0) = drone_pose_->pose.position.y;
        state_y(1) = drone_velocity_->twist.linear.y;
        state_y(2)= load_angle_-> angle.angle_y;
        //state_y(3)= load_imu_->angular_velocity.y;

        update_load_angular_velocity();
        //updateTargetPositionGradually();
        //state_x(0) = //drone_pose_->pose.position.x - current_target_position_(0);
        //state_y(0) = //drone_pose_->pose.position.y - current_target_position_(1);
        RCLCPP_INFO(this->get_logger(), "State x: [%f, %f, %f, %f]", state_x(0), state_x(1), state_x(2), state_x(3));
        RCLCPP_INFO(this->get_logger(), "State y: [%f, %f, %f, %f]", state_y(0), state_y(1), state_y(2), state_y(3));
    }

    void compute_attitude(double control_input_x, double control_input_y){
        double g = 9.81;
        double acceleration_x = control_input_x/system_mass;
        double acceleration_y = control_input_y/system_mass;
        roll = -acceleration_y/g;
        pitch = +acceleration_x/g;
        // Saturation limits
        double max_tilt_angle = 0.2617993878;  // 15 degrees in radians

        // Apply saturation
        roll = std::max(std::min(roll, max_tilt_angle), -max_tilt_angle);
        pitch = std::max(std::min(pitch, max_tilt_angle), -max_tilt_angle);

        RCLCPP_INFO(this->get_logger(), "ENU - DEBUG: roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);
    }

    void control(){
        if (!drone_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Drone pose not available for control");
        return;
        }
        create_state_vector();
        control_input_x = -K.dot(state_x);
        control_input_y = -K.dot(state_y);
        publishStateVector();
        yaw = getYawFromQuaternion(drone_pose_->pose.orientation);
        auto rotated_inputs = rotateControlInputs(control_input_x, control_input_y, yaw);

        double control_input_x_rotated = rotated_inputs.first; 
        double control_input_y_rotated = rotated_inputs.second;
        RCLCPP_INFO(this->get_logger(), "Control input x: %.2f",control_input_x_rotated);
        RCLCPP_INFO(this->get_logger(), "Control input y: %.2f",control_input_y_rotated);
        compute_attitude(control_input_x_rotated,control_input_y_rotated);
        publish_control(roll, pitch, yaw);
    }

void publish_control(double roll, double pitch, double yaw){
        if (!drone_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Drone pose not available for control");
        return;
    }

    double desired_altitude = 10;
    double altitude_error = desired_altitude - drone_pose_->pose.position.z;
    double dt = 0.02;
    double thrust_adjustment = pid.compute(altitude_error, dt);
    double base_thrust = 0.6;
    double new_thrust = base_thrust + thrust_adjustment;

    new_thrust = std::max(0.0, std::min(1.0, new_thrust));

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    quaternion.normalize();

    mavros_msgs::msg::AttitudeTarget attitude_msg;
    attitude_msg.header.stamp = this->get_clock()->now();
    attitude_msg.header.frame_id = "base_link";
    attitude_msg.thrust = new_thrust;
    attitude_msg.orientation.x = quaternion.x();
    attitude_msg.orientation.y = quaternion.y();
    attitude_msg.orientation.z = quaternion.z();
    attitude_msg.orientation.w = quaternion.w();

    attitude_publisher_->publish(attitude_msg);
}

void updateTargetPositionGradually() {
    Eigen::Vector3d position_difference = desired_target_position_ - current_target_position_;

    if (position_difference.norm() > step_size_) {
        position_difference = step_size_ * position_difference.normalized();
    }
    current_target_position_ += position_difference;
}

std::pair<double, double> rotateControlInputs(double input_x, double input_y, double yaw) {
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setRPY(0, 0, yaw);
    tf2::Vector3 inputs(input_x, input_y, 0);
    tf2::Vector3 rotated_inputs = rotation_matrix.transpose() * inputs;
    return {rotated_inputs.x(), rotated_inputs.y()};
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void publishStateVector(){
    std_msgs::msg::Float64MultiArray state_msg;
    state_msg.data = {state_x(0), state_x(1), state_x(2), state_x(3), state_y(0), state_y(1), state_y(2), state_y(3)};
    state_publisher_->publish(state_msg);

}

void loadLQRParams()
{
    std::string file_path = "/home/luky/mavros_ros2_ws/lqr_controller/src/params.yaml";
    YAML::Node config = YAML::LoadFile(file_path);

    if (config["lqr_params"])
    {
        std::vector<double> K_ = config["lqr_params"]["K_x"].as<std::vector<double>>();
        if (K_.size() == 4)
        {
            K = Eigen::VectorXd(4);

            for (int i = 0; i < 4; ++i)
            {
                K(i) = K_[i];
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid size for K in LQR parameters");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "LQR parameters not found in YAML file");
    }
}



    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr load_imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<angle_stamped_msg::msg::AngleStamped>::SharedPtr load_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr drone_velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr K_matrix_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_publisher_;
    geometry_msgs::msg::PoseStamped::SharedPtr drone_pose_;
    angle_stamped_msg::msg::AngleStamped::SharedPtr load_angle_;
    sensor_msgs::msg::Imu::SharedPtr load_imu_;
    geometry_msgs::msg::TwistStamped::SharedPtr drone_velocity_;
    Eigen::VectorXd K;
    Eigen::VectorXd state_x;
    Eigen::VectorXd state_y;
    double control_input_x;
    double control_input_y;
    double system_mass;
    double pitch = 0;
    double yaw =0;
    double roll = 0;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_publisher_;
    PIDController pid = PIDController(1.2, 0.1, 0.45, -1,1);
    Eigen::Vector3d current_target_position_{0, 0, 10}; 
    Eigen::Vector3d desired_target_position_{10, 0, 10};
    double step_size_ = 0.025;

};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRController>());
    rclcpp::shutdown();
    return 0;
}
