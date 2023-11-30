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
        attitude_publisher_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 20);

        state_x = Eigen::VectorXd::Zero(4);
        state_y = Eigen::VectorXd::Zero(4);
        loadLQRParams();
        system_mass = 2.25;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),  
            std::bind(&LQRController::control, this)
        );
    }

private:

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

    // Convert geometry_msgs::Quaternion to tf2::Quaternion
    tf2::Quaternion drone_orientation_tf2, imu_orientation_tf2;
    tf2::fromMsg(drone_pose_->pose.orientation, drone_orientation_tf2);
    tf2::fromMsg(load_imu_->orientation, imu_orientation_tf2);

    // Normalize the quaternions
    drone_orientation_tf2.normalize();
    imu_orientation_tf2.normalize();

    // Combine the orientations to find the IMU's orientation in the global frame
    tf2::Quaternion imu_orientation_global = drone_orientation_tf2 * imu_orientation_tf2;
    imu_orientation_global.normalize();

    // Convert quaternion to rotation matrix
    Eigen::Matrix3d eigen_rotation_matrix;
    tf2::Matrix3x3 rotation_matrix(imu_orientation_global);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            eigen_rotation_matrix(i, j) = rotation_matrix[i][j];

    // Obtain angular velocity from IMU data
    Eigen::Vector3d local_angular_velocity(
        load_imu_->angular_velocity.x,
        load_imu_->angular_velocity.y,
        load_imu_->angular_velocity.z
    );

    // Transform angular velocity from the IMU's frame to the global frame
    Eigen::Vector3d global_angular_velocity = eigen_rotation_matrix.inverse() * local_angular_velocity;

    // Update state vector
    state_x(3) = global_angular_velocity.x();
    state_y(3) = global_angular_velocity.y();
}

    void create_state_vector(){

        if (!drone_pose_ || !load_imu_ || !load_angle_ || !drone_velocity_) {
        RCLCPP_ERROR(this->get_logger(), "Missing required data for state vector creation");
        return;
    }

        state_x(0) = drone_pose_->pose.position.x;
        state_x(1) = drone_velocity_->twist.linear.x;
        state_x(2)= load_angle_-> angle.angle_x;
        state_x(3)= load_imu_->angular_velocity.x;


        state_y(0) = drone_pose_->pose.position.y;
        state_y(1) = drone_velocity_->twist.linear.y;
        state_y(2)= load_angle_-> angle.angle_y;
        state_y(3)= load_imu_->angular_velocity.y;
        update_load_angular_velocity();

        RCLCPP_INFO(this->get_logger(), "State x: [%f, %f, %f, %f]", state_x(0), state_x(1), state_x(2), state_x(3));
        RCLCPP_INFO(this->get_logger(), "State y: [%f, %f, %f, %f]", state_y(0), state_y(1), state_y(2), state_y(3));

    }

    void compute_attitude(){
        double g = 9.81;
        double acceleration_x = control_input_x/system_mass;
        double acceleration_y = control_input_y/system_mass;
        roll = -acceleration_y/g;
        pitch = -acceleration_x/g;
        // Saturation limits
        double max_tilt_angle = 0.698;  // 40 degrees in radians

        // Apply saturation
        roll = std::max(std::min(roll, max_tilt_angle), -max_tilt_angle);
        pitch = std::max(std::min(pitch, max_tilt_angle), -max_tilt_angle);
                // Convert from ENU to NED for roll and pitch
        //roll = -roll;
        //pitch = -pitch;
        double temp = roll; 
        roll = pitch;
        pitch = temp;

        RCLCPP_WARN(this->get_logger(), "DEBUG: roll: %.2f, pitch: %.2f", roll, pitch);
    }

    void control(){
        if (!drone_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Drone pose not available for control");
        return;
        }
        create_state_vector();
        updateTargetPositionGradually();
        state_x(0) = drone_pose_->pose.position.x - current_target_position_(0);
        state_y(0) = drone_pose_->pose.position.y - current_target_position_(1);
        control_input_x = (-K_x_ * state_x).coeff(0,0);
        control_input_y = (-K_y_ * state_y).coeff(0,0);
        yaw = getYawFromQuaternion(drone_pose_->pose.orientation);
        auto rotated_inputs = rotateControlInputs(control_input_x, control_input_y, yaw);

        control_input_x = rotated_inputs.first;
        control_input_y = rotated_inputs.second;
        RCLCPP_INFO(this->get_logger(), "Control input x: %.2f",control_input_x);
        RCLCPP_INFO(this->get_logger(), "Control input y: %.2f",control_input_y);
        compute_attitude();
        publish_control(roll, pitch, yaw);
    }

void publish_control(double roll, double pitch, double yaw){
        if (!drone_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Drone pose not available for control");
        return;
    }
    double desired_altitude = 10;
    double altitude_error = desired_altitude - drone_pose_->pose.position.z;

    double dt = 0.05;
    double thrust_adjustment = pid.compute(altitude_error, dt);

    double base_thrust = 0.6;
    double new_thrust = base_thrust + thrust_adjustment;

    new_thrust = std::max(0.0, std::min(1.0, new_thrust));
    tf2::Quaternion new_drone_orientation_NED;

    new_drone_orientation_NED.setRPY(roll,pitch, yaw); //this should transform from ENU to NED
    RCLCPP_ERROR(this->get_logger(), "yaw: %.2f",yaw);
    new_drone_orientation_NED.normalize();
    mavros_msgs::msg::AttitudeTarget attitude_msg;
    attitude_msg.header.stamp = this->get_clock()->now();
    attitude_msg.header.frame_id = "base_link";
    attitude_msg.thrust = new_thrust;
    attitude_msg.orientation.x = new_drone_orientation_NED.x();
    attitude_msg.orientation.y = new_drone_orientation_NED.y();
    attitude_msg.orientation.z = new_drone_orientation_NED.z();
    attitude_msg.orientation.w = new_drone_orientation_NED.w();

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
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(yaw), -sin(yaw),
                       sin(yaw),  cos(yaw);

    Eigen::Vector2d inputs(input_x, input_y);
    Eigen::Vector2d rotated_inputs = rotation_matrix * inputs;

    return {rotated_inputs(0), rotated_inputs(1)};
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void loadLQRParams()
    {
        
        std::string file_path = "/home/luky/mavros_ros2_ws/src/lqr_controller/src/params.yaml";
        YAML::Node config = YAML::LoadFile(file_path);

        if (config["lqr_params"])
        {
            std::vector<double> K_x = config["lqr_params"]["K_x"].as<std::vector<double>>();
            std::vector<double> K_y = config["lqr_params"]["K_y"].as<std::vector<double>>();
            if (K_x.size() == 4 and K_y.size() == 4)
            {
                K_x_ = (Eigen::MatrixXd(1, 4) << K_x[0], K_x[1], K_x[2], K_x[3]).finished();
                K_y_ = (Eigen::MatrixXd(1, 4) << K_y[0], K_y[1], K_y[2], K_y[3]).finished();
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
    geometry_msgs::msg::PoseStamped::SharedPtr drone_pose_;
    angle_stamped_msg::msg::AngleStamped::SharedPtr load_angle_;
    sensor_msgs::msg::Imu::SharedPtr load_imu_;
    geometry_msgs::msg::TwistStamped::SharedPtr drone_velocity_;
    Eigen::MatrixXd K_x_;
    Eigen::MatrixXd K_y_;
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
    double step_size_ = 0.05;

};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRController>());
    rclcpp::shutdown();
    return 0;
}
