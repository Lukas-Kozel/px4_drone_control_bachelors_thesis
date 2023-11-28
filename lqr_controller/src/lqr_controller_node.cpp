#include "rclcpp/rclcpp.hpp"
#include "PIDController.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Eigen/Dense"
#include <cmath>
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

auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10
));
qos2.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // Match the publisher's QoS settings
       load_pose_subscriber_ = this->create_subscription<load_pose_stamped::msg::LoadPoseStamped>(
            "/ros_load_pose", 10, std::bind(&LQRController::on_load_pose_received, this, std::placeholders::_1));
       /* drone_pose_subscriber_ = this->create_subscription<drone_pose_stamped::msg::DronePoseStamped>(
            "/ros_drone_pose", 10, std::bind(&LQRController::on_drone_pose_received, this, std::placeholders::_1));*/
        drone_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&LQRController::on_drone_pose_received, this, std::placeholders::_1));
        load_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/load_imu", 10, std::bind(&LQRController::on_load_imu_received, this, std::placeholders::_1));
        load_angle_subscriber_ = this->create_subscription<angle_stamped_msg::msg::AngleStamped>(
            "/load_angle", 10, std::bind(&LQRController::on_load_angle_received, this, std::placeholders::_1));
        drone_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", qos, std::bind(&LQRController::on_drone_velocity_received, this, std::placeholders::_1));
        attitude_publisher_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 20);
        drone_state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos2, std::bind(&LQRController::on_drone_state_received, this, std::placeholders::_1));

        state_x = Eigen::VectorXd::Zero(4);
        state_y = Eigen::VectorXd::Zero(4);
        K_x_ = (Eigen::MatrixXd(1, 4) <<   0.1000 ,   1.2929 ,  10.8637 ,   3.9498).finished(); //0.1,2.1449,7.3917,3.0167 4.0,5.2133,6.35,2.0023 0.2
        K_y_ = (Eigen::MatrixXd(1, 4) <<   0.1000 ,   1.2929 ,  10.8637 ,   3.9498).finished(); // 0.1000 ,   1.2929 ,  10.8637 ,   3.9498 // 0.1000   , 2.3641  ,  9.8428  ,  3.5739
        system_mass = 2.25;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  
            std::bind(&LQRController::control, this)
        );
    }

private:
 void on_load_pose_received(const load_pose_stamped::msg::LoadPoseStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    load_pose_ = msg;
}
/*
    void on_drone_pose_received(const drone_pose_stamped::msg::DronePoseStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    drone_pose_ = msg;
}
*/

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
    state_x(3)=0;
    state_y(3) = global_angular_velocity.y();
    state_y(3)=0;
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
        //state_y(0) = load_pose_->pose.position.y;
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
        double yaw = getYawFromQuaternion(drone_pose_->pose.orientation);
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
    double desired_altitude = 10; // Set your desired altitude
    double current_altitude = drone_pose_->pose.position.z;
    double altitude_error = desired_altitude - current_altitude;

    double dt = 0.05; // Calculate time difference since last control call
    double thrust_adjustment = pid.compute(altitude_error, dt);

    // Use thrust_adjustment to set the thrust in your control message
    double base_thrust = 0.6; // Base thrust needed to hover (adjust as needed)
    double new_thrust = base_thrust + thrust_adjustment;

    // Make sure new_thrust is within valid range
    new_thrust = std::max(0.0, std::min(1.0, new_thrust));
    tf2::Quaternion combined_orientation;

    combined_orientation.setRPY(roll,-pitch, -yaw); //this should transform from ENU to NED
    RCLCPP_ERROR(this->get_logger(), "yaw: %.2f",yaw);
    combined_orientation.normalize();
    mavros_msgs::msg::AttitudeTarget attitude_msg;
    attitude_msg.header.stamp = this->get_clock()->now();
    attitude_msg.header.frame_id = "base_link";
    attitude_msg.thrust = new_thrust;
    attitude_msg.orientation.x = combined_orientation.x();
    attitude_msg.orientation.y = combined_orientation.y();
    attitude_msg.orientation.z = combined_orientation.z();
    attitude_msg.orientation.w = combined_orientation.w();

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

   void on_drone_state_received(const mavros_msgs::msg::State::SharedPtr msg)
    {
        if (msg == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Received null message in on_drone_state_received");
            return;
        }
        offboard_mode_ = (msg->mode== "OFFBOARD");
    }


    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<load_pose_stamped::msg::LoadPoseStamped>::SharedPtr load_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr load_imu_subscriber_;
    //rclcpp::Subscription<drone_pose_stamped::msg::DronePoseStamped>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<angle_stamped_msg::msg::AngleStamped>::SharedPtr load_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr drone_velocity_subscriber_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr drone_state_subscriber_;
    load_pose_stamped::msg::LoadPoseStamped::SharedPtr load_pose_;
    //drone_pose_stamped::msg::DronePoseStamped::SharedPtr drone_pose_;
    geometry_msgs::msg::PoseStamped::SharedPtr drone_pose_;
    angle_stamped_msg::msg::AngleStamped::SharedPtr load_angle_;
    sensor_msgs::msg::Imu::SharedPtr load_imu_;
    sensor_msgs::msg::Imu::SharedPtr drone_imu_;
    geometry_msgs::msg::TwistStamped::SharedPtr drone_velocity_;
    Eigen::MatrixXd K_x_;
    Eigen::MatrixXd K_y_;
    Eigen::VectorXd state_x;
    Eigen::VectorXd state_y;
    double control_input_x;
    double control_input_y;
    double system_mass;
    double pitch = 0;
    double roll = 0;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_publisher_;
    Eigen::Vector3d previous_load_angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d load_angular_acceleration_ = Eigen::Vector3d::Zero();
    rclcpp::Time last_time_ = this->get_clock()->now();  // Initialize with current time
    bool offboard_mode_ = false;
    PIDController pid = PIDController(1.2, 0.1, 0.45, -1,1); //0.15, 0.005, 0.08, -1,1 regulating on 8.9    0.8, 0.07, 0.3, -1,1 almost perfect
    Eigen::Vector3d current_target_position_{0, 0, 10}; // Current target for gradual movement
    Eigen::Vector3d desired_target_position_{-10, 10, 10}; // Final desired position
    double step_size_ = 0.05; // Step size for gradual movement, adjust as needed

};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRController>());
    rclcpp::shutdown();
    return 0;
}
