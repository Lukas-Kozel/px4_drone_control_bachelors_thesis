#include "rclcpp/rclcpp.hpp"
#include "PIDController.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <random>
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
#include <cstdlib>


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
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
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
        getInputParameters();
        setOffboardMode();
        system_mass = 2.25;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  
            std::bind(&LQRController::control, this)
        );
    }

private:

void on_K_matrix_received(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if(msg == nullptr) {
            RCLCPP_INFO(this->get_logger(), "Message is null");
            return;
        }
        if(K.size() != static_cast<int>(msg->data.size())) {
            RCLCPP_INFO(this->get_logger(), "Size mismatch, K not updated");
            return;
        }
        Eigen::VectorXd incomingK = Eigen::Map<const Eigen::VectorXd>(msg->data.data(), msg->data.size());
        if(!recording_started) startROS2BagRecording();
        K_matrix_loaded=true;
        if(K == incomingK) {
            RCLCPP_INFO(this->get_logger(), "K is equal to incoming data, no update needed");
        } else {
            K = incomingK;
            RCLCPP_INFO(this->get_logger(), "K updated with new data");
        }
    }
    

void on_drone_pose_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    double pose_stddev = 0.1;
    init_pose_x =msg->pose.position.x;
    init_pose_y =msg->pose.position.y; 
    msg->pose.position.x += generateWhiteNoise(pose_stddev);
    msg->pose.position.y += generateWhiteNoise(pose_stddev);
    msg->pose.position.z += generateWhiteNoise(pose_stddev);
    drone_pose_ = msg;
}

    void on_load_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
    {        
    if (msg == nullptr) {
        return;
    }

    // Specify standard deviations for IMU readings based on their range
    double angular_velocity_stddev = 0.02;

    // Add noise to angular velocity
    msg->angular_velocity.x += generateWhiteNoise(angular_velocity_stddev);
    msg->angular_velocity.y += generateWhiteNoise(angular_velocity_stddev);
    msg->angular_velocity.z += generateWhiteNoise(angular_velocity_stddev);


    load_imu_ = msg;   
    }
    void on_load_angle_received(const angle_stamped_msg::msg::AngleStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    double angle_stddev = 0.0523598776; //3 deg
    original_angle_x = msg->angle.angle_x;
    original_angle_y = msg->angle.angle_y;
    msg->angle.angle_x += generateWhiteNoise(angle_stddev);
    msg->angle.angle_y += generateWhiteNoise(angle_stddev);
    load_angle_ = msg;
}
    void on_drone_velocity_received(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }
    // Specify standard deviations for IMU readings based on their range
    double velocity_stddev = 0.02;
    msg->twist.linear.x += generateWhiteNoise(velocity_stddev);
    msg->twist.linear.y += generateWhiteNoise(velocity_stddev);
    msg->twist.linear.z += generateWhiteNoise(velocity_stddev);

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
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix = tf2::Matrix3x3(load_orientation);

    tf2::Vector3 local_angular_velocity(
        load_imu_->angular_velocity.x,
        load_imu_->angular_velocity.y,
        load_imu_->angular_velocity.z
    );
    tf2::Vector3 rotated_load_imu = rotation_matrix.transpose() * local_angular_velocity;
    state_x(3) = rotated_load_imu.x();
    state_y(3) = rotated_load_imu.y();
}

    void create_state_vector(){
        if (!drone_pose_ || !load_imu_ || !load_angle_ || !drone_velocity_) {
        RCLCPP_ERROR(this->get_logger(), "Missing required data for state vector creation");
        return;
    }   
        original_position_x  = init_pose_x - current_target_position_(0);
        original_position_y  = init_pose_y - current_target_position_(1);
        state_x(0) = drone_pose_->pose.position.x - current_target_position_(0);
        state_x(1) = drone_velocity_->twist.linear.x;
        state_x(2)= load_angle_->angle.angle_x;

        state_y(0) = drone_pose_->pose.position.y - current_target_position_(1);
        state_y(1) = drone_velocity_->twist.linear.y;
        state_y(2)= load_angle_->angle.angle_y;
        //state_x(3) = 0;
        //state_y(3) = 0;
        update_load_angular_velocity();
        RCLCPP_INFO(this->get_logger(), "State x: [%f, %f, %f, %f]", state_x(0), state_x(1), state_x(2), state_x(3));
        RCLCPP_INFO(this->get_logger(), "State y: [%f, %f, %f, %f]", state_y(0), state_y(1), state_y(2), state_y(3));
    }

    void compute_attitude(double control_input_x, double control_input_y){
        double g = 9.81;
        double acceleration_x = control_input_x/system_mass;
        double acceleration_y = control_input_y/system_mass;
        roll = -acceleration_y/g;
        pitch = +acceleration_x/g;
        double max_tilt_angle = 0.2617993878; 
        roll = std::max(std::min(roll, max_tilt_angle), -max_tilt_angle);
        pitch = std::max(std::min(pitch, max_tilt_angle), -max_tilt_angle);
    }

void control(){
        setOffboardMode();
        if (!drone_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Drone pose not available for control");
        return;
        }
        if (trajectory_type == "circle") {
            circleTrajectorySetup();
            updateCircleTargetPosition();
        }
        else{
            updateTargetPositionGradually();
        }
        create_state_vector();
        RCLCPP_INFO(this->get_logger(), "K: [%f, %f, %f, %f]", K(0), K(1), K(2), K(3));
        control_input_x = -K.dot(state_x);
        control_input_y = -K.dot(state_y);
        publishStateVector();
        calculateRotation();
        tf2::Quaternion drone_orientation(drone_pose_->pose.orientation.x, 
                        drone_pose_->pose.orientation.y, 
                        drone_pose_->pose.orientation.z, 
                        drone_pose_->pose.orientation.w);
                        
        auto rotated_inputs = rotateControlInputs(control_input_x, control_input_y, drone_orientation);
        double control_input_x_rotated = rotated_inputs.first; 
        double control_input_y_rotated = rotated_inputs.second;
        RCLCPP_INFO(this->get_logger(), "DEBUG final control inputs: %.2f, %.2f",control_input_x_rotated,control_input_y_rotated);
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
    double dt = 0.033;
    double thrust_adjustment = pid.compute(altitude_error, dt);
    double base_thrust = 0.6;
    double new_thrust = base_thrust + thrust_adjustment;

    new_thrust = std::max(0.0, std::min(1.0, new_thrust));
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch,rotation_towards_waypoint);
    quaternion.normalize();
    RCLCPP_INFO(this->get_logger(), "rotation towards waypoint: %.2f",yaw);
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


void circleTrajectorySetup() {
    if (!initial_position_captured_ && drone_pose_) {
        initial_position_ = Eigen::Vector3d(
            drone_pose_->pose.position.x,
            drone_pose_->pose.position.y,
            drone_pose_->pose.position.z
        );
        circle_center_ = Eigen::Vector3d(circle_center_offset_(0),circle_center_offset_(1), 0); // radius, 0,0

        initial_angle = atan2(initial_position_.y() - circle_center_.y(), initial_position_.x() - circle_center_.x());
        current_target_position_ = circle_center_ + Eigen::Vector3d(radius * cos(initial_angle), radius * sin(initial_angle), 0);
        initial_position_captured_ = true;
    }
}


void updateCircleTargetPosition() {
    if (!initial_position_captured_) return;
    if (trajectory_type != "circle") return;
        if (!drone_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Drone pose data not available");
        return;
    }
    static double angle = initial_angle; 
    double angle_increment = M_PI / 180.0 * 0.15;

    angle += angle_increment;
    if (angle >= 2 * M_PI){  
        angle -= 2 * M_PI;
    }

    current_target_position_(0) = circle_center_(0) + radius * cos(angle);
    current_target_position_(1) = circle_center_(1) + radius * sin(angle);
}


double normalizeAngle(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) angle += 2.0 * M_PI;
    angle -= M_PI;
    return angle;
}


void calculateRotation(){
    if(trajectory_type=="waypoint"){
        calculateRotationTowardsWaypoint();
    }
    else if(trajectory_type=="circle"){
        calculateRotationForCircle();
    }
    else{
        rotation_towards_waypoint = yaw;
    }
}


void calculateRotationTowardsWaypoint() {
    double targetAngle = atan2(current_target_position_(1) - drone_pose_->pose.position.y, 
                               current_target_position_(0) - drone_pose_->pose.position.x);
    targetAngle = normalizeAngle(targetAngle);
    
    double distanceToTarget = sqrt(pow(current_target_position_(0) - drone_pose_->pose.position.x, 2) +
                                   pow(current_target_position_(1) - drone_pose_->pose.position.y, 2));
    RCLCPP_INFO(this->get_logger(), "distanceToTarget: %f",distanceToTarget);                
    if (distanceToTarget > 1.25) {
        rotation_towards_waypoint = targetAngle;
    }
}


void calculateRotationForCircle() {
    double targetAngle = atan2(current_target_position_(1) - drone_pose_->pose.position.y, 
                               current_target_position_(0) - drone_pose_->pose.position.x);
    targetAngle = normalizeAngle(targetAngle);
    rotation_towards_waypoint = targetAngle;
}


std::pair<double, double> rotateControlInputs(double input_x, double input_y, tf2::Quaternion drone_orientation) {
    drone_orientation.normalize();
    tf2::Matrix3x3 rotation_matrix = tf2::Matrix3x3(drone_orientation);
    tf2::Vector3 inputs(input_x, input_y, 0);
    tf2::Vector3 rotated_inputs = rotation_matrix.transpose() * inputs;
    return {rotated_inputs.x(), rotated_inputs.y()};
}


void publishStateVector(){
    std_msgs::msg::Float64MultiArray state_msg;
    //state_msg.data = {state_x(0), state_x(1), state_x(2), state_x(3), state_y(0), state_y(1), state_y(2), state_y(3),drone_pose_->pose.position.x,drone_pose_->pose.position.y,original_angle_x,original_angle_y, original_position_x,original_position_y};
    state_msg.data = {state_x(0), state_x(1), state_x(2), state_x(3), state_y(0), state_y(1), state_y(2), state_y(3),init_pose_x,init_pose_y,original_angle_x,original_angle_y, original_position_x,original_position_y};
    state_publisher_->publish(state_msg);
}


void setOffboardMode(){
    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "OFFBOARD";
    auto set_mode_future = set_mode_client_->async_send_request(set_mode_request);
}


void getInputParameters() {
    this->declare_parameter<std::string>("trajectory_type", "waypoint");
    double waypoint_x, waypoint_y;
    this->declare_parameter<double>("waypoint_x", 0.0);
    this->declare_parameter<double>("waypoint_y", 0.0);
    this->declare_parameter<double>("radius", 0.0);

    this->get_parameter("trajectory_type", trajectory_type);

    if (trajectory_type == "waypoint") {
        this->get_parameter("waypoint_x", waypoint_x);
        this->get_parameter("waypoint_y", waypoint_y);
        desired_target_position_ = Eigen::Vector3d(waypoint_x, waypoint_y, 10.0);
        RCLCPP_INFO(this->get_logger(), "Waypoint X: %f, Waypoint Y: %f", waypoint_x, waypoint_y);
    } else if (trajectory_type == "circle") {
        this->get_parameter("radius", radius);
        RCLCPP_INFO(this->get_logger(),"radius %f", radius);
        double center_x = 0;
        double center_y = 0;
        double center_z = 10.0;
        double initial_angle = 0;
        current_target_position_ = Eigen::Vector3d(
            center_x + radius * cos(initial_angle),
            center_y + radius * sin(initial_angle),
            center_z
        );
        circle_center_offset_=Eigen::Vector3d(radius, 0, 10.0);
        RCLCPP_INFO(this->get_logger(), "Circle Center X: %f, Y: %f, Radius: %f", center_x, center_y, radius);
    }
}

void startROS2BagRecording() {
    recording_started = true;
    // Specify the desired output directory and filename for the ROS bag
    const char* outputDirectory = "/home/luky/mavros_ros2_ws/rosbags";
    const char* bagName = "waypoint_trajectory"; // The name for your ROS bag without extension

    // Construct the command with the specified output directory and bag name
    // Adjust the command according to your terminal emulator and file path requirements
    std::string command = "gnome-terminal -- bash -c 'ros2 bag record -a -o ";
    command += outputDirectory;
    command += "/";
    command += bagName;
    command += "; exec bash'";

    // Execute the command
    int result = system(command.c_str());

    if(result != 0) {
        // Handle error
    }
}

double generateWhiteNoise(double stddev) {
    static std::random_device rd{};
    static std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, stddev};
    //return 0;
    return d(gen);
}



    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr load_imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<angle_stamped_msg::msg::AngleStamped>::SharedPtr load_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr drone_velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr K_matrix_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_publisher_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    geometry_msgs::msg::PoseStamped::SharedPtr drone_pose_;
    angle_stamped_msg::msg::AngleStamped::SharedPtr load_angle_;
    sensor_msgs::msg::Imu::SharedPtr load_imu_;
    geometry_msgs::msg::TwistStamped::SharedPtr drone_velocity_;
    Eigen::VectorXd K=Eigen::VectorXd::Zero(4);
    Eigen::VectorXd state_x;
    Eigen::VectorXd state_y;
    double control_input_x;
    double control_input_y;
    double system_mass;
    double pitch = 0;
    double yaw =0;
    double roll = 0;
    double circle_counter=0;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_publisher_;
    PIDController pid = PIDController(1.2, 0.1, 0.45, -1,1);
    Eigen::Vector3d current_target_position_{0, 0, 10};
    Eigen::Vector3d desired_target_position_;
    double step_size_ = 0.0075;
    std::string trajectory_type;
    double radius=0;
    double rotation_towards_waypoint=0;
    Eigen::Vector3d initial_position_;
    bool initial_position_captured_ = false;
    Eigen::Vector3d circle_center_;
    Eigen::Vector3d circle_center_offset_;
    double initial_angle=0;
    bool recording_started=false;
    bool K_matrix_loaded=false;
    double original_angle_x;
    double original_angle_y;
    double original_position_x;
    double original_position_y;
    double init_pose_x;
    double init_pose_y;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRController>());
    rclcpp::shutdown();
    return 0;
}
