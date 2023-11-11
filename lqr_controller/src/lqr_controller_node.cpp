#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"
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
    10  // The depth of the QoS history, similar to your original code
));
qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10  // The depth of the QoS history, similar to your original code
));
qos2.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // Match the publisher's QoS settings
       load_pose_subscriber_ = this->create_subscription<load_pose_stamped::msg::LoadPoseStamped>(
            "/ros_load_pose", 10, std::bind(&LQRController::on_load_pose_received, this, std::placeholders::_1));
        drone_pose_subscriber_ = this->create_subscription<drone_pose_stamped::msg::DronePoseStamped>(
            "/ros_drone_pose", 10, std::bind(&LQRController::on_drone_pose_received, this, std::placeholders::_1));
        load_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/load_imu", 10, std::bind(&LQRController::on_load_imu_received, this, std::placeholders::_1));
        load_angle_subscriber_ = this->create_subscription<angle_stamped_msg::msg::AngleStamped>(
            "/load_angle", 10, std::bind(&LQRController::on_load_angle_received, this, std::placeholders::_1));
        drone_velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_body", qos, std::bind(&LQRController::on_drone_velocity_received, this, std::placeholders::_1));
        attitude_publisher_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 20);
        drone_state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos2, std::bind(&LQRController::on_drone_state_received, this, std::placeholders::_1));

        state_x = Eigen::VectorXd::Zero(4);
        state_y = Eigen::VectorXd::Zero(4);
        K_x_ = (Eigen::MatrixXd(1, 4) << 2.2361, 3.9722, 2.9874, 1.6828).finished(); //7.0711, 8.8439, 17.8884, 5.3427 //158.1139 , 105.3818, 253.0351, 3.5602
        K_y_ = (Eigen::MatrixXd(1, 4) << 2.2361, 3.9722, 2.9874, 1.6828).finished(); //2.2361, 3.9722, 2.9874, 1.6828
        system_mass = 3;
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
    void on_drone_pose_received(const drone_pose_stamped::msg::DronePoseStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        //RCLCPP_ERROR(this->get_logger(), "Received null message in on_drone_pose_received");
        return;
    }
    drone_pose_ = msg;
   // RCLCPP_INFO(this->get_logger(), "Received drone pose message");
}
    void on_load_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
    {        
    if (msg == nullptr) {
      //  RCLCPP_ERROR(this->get_logger(), "Received null message in on_load_imu_received");
        return;
    }
    load_imu_ = msg;
    //RCLCPP_INFO(this->get_logger(), "Received load imu message");
    
    }
    void on_load_angle_received(const angle_stamped_msg::msg::AngleStamped::SharedPtr msg)
{
    if (msg == nullptr) {
      //  RCLCPP_ERROR(this->get_logger(), "Received null message in on_load_angle_received");
        return;
    }
    load_angle_ = msg;
    //RCLCPP_INFO(this->get_logger(), "Received load angle message");
}
    void on_drone_velocity_received(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    if (msg == nullptr) {
      //  RCLCPP_ERROR(this->get_logger(), "Received null message in on_drone_velocity_received");
        return;
    }
    drone_velocity_ = msg;
    //RCLCPP_INFO(this->get_logger(), "Received drone velocity message");
}


    void create_state_vector(){

        if (!drone_pose_ || !load_imu_ || !load_angle_ || !drone_velocity_) {
        RCLCPP_ERROR(this->get_logger(), "Missing required data for state vector creation");
        return;
    }

       /* tf2::Matrix3x3 rotation_matrix(tf2::Quaternion(
        drone_pose_->pose.orientation.x,
        drone_pose_->pose.orientation.y,
        drone_pose_->pose.orientation.z,
        drone_pose_->pose.orientation.w
        ));
        */
        //tf2::Vector3 position_world(drone_pose_->pose.position.x, drone_pose_->pose.position.y, 0);
        //tf2::Vector3 velocity_world(drone_velocity_->twist.linear.x, drone_velocity_->twist.linear.y, 0);

        //::Vector3 position_drone = rotation_matrix.inverse() * position_world;
        //tf2::Vector3 velocity_drone = rotation_matrix.inverse() * velocity_world;

        //state_x(0) = drone_pose_->pose.position.x;
        state_x(0) = load_pose_->pose.position.x;
        state_x(1) = drone_velocity_->twist.linear.x;
        state_x(2)= load_angle_-> angle.angle_x;
        state_x(3)= load_imu_->angular_velocity.x;

        //state_y(0) = drone_pose_->pose.position.y;
        state_y(0) = load_pose_->pose.position.y;
        state_y(1) = drone_velocity_->twist.linear.x;
        state_y(2)= load_angle_-> angle.angle_y;
        state_y(3)= load_imu_->angular_velocity.y;

        RCLCPP_INFO(this->get_logger(), "State x: [%f, %f, %f, %f]", state_x(0), state_x(1), state_x(2), state_x(3));
        RCLCPP_INFO(this->get_logger(), "State y: [%f, %f, %f, %f]", state_y(0), state_y(1), state_y(2), state_y(3));

    }

    void compute_attitude(){
        double g = 9.81;
        double acceleration_x = control_input_x/system_mass;
        double acceleration_y = control_input_y/system_mass;

        //roll = acos((g)/(2*acceleration_y)); // roll is around axis x
        //pitch = acos((g)/(2*acceleration_x)); // pitch is around axis y
        roll = -acceleration_y/g;
        pitch = -acceleration_x/g;

        RCLCPP_WARN(this->get_logger(),"DEBUG: roll: %.2f, pitch: %.2f",roll,pitch);
    }

    void control(){
        create_state_vector();
        control_input_x = (-K_x_ * state_x).coeff(0,0);
        control_input_y = (-K_y_ * state_y).coeff(0,0);
        RCLCPP_INFO(this->get_logger(), "Control input x: %.2f",control_input_x);
        RCLCPP_INFO(this->get_logger(), "Control input y: %.2f",control_input_y);
        compute_attitude();
        publish_control(roll, pitch);
    }

void publish_control(double roll, double pitch){
    if (!drone_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Drone pose not available for control");
        return;
    }

    // Current drone orientation
    tf2::Quaternion current_orientation(
        drone_pose_->pose.orientation.x,
        drone_pose_->pose.orientation.y,
        drone_pose_->pose.orientation.z,
        drone_pose_->pose.orientation.w
    );

    // Desired rotation in drone frame
    tf2::Quaternion desired_rotation;
    desired_rotation.setRPY(roll, pitch, 0);

    // Combine orientations
    tf2::Quaternion combined_orientation = current_orientation * desired_rotation;
    combined_orientation.normalize(); // Ensure the quaternion is normalized

    // Prepare attitude message
    mavros_msgs::msg::AttitudeTarget attitude_msg;
    attitude_msg.header.stamp = this->get_clock()->now();
    attitude_msg.header.frame_id = "base_link";
    attitude_msg.thrust = 0.95; // Set thrust as needed
    attitude_msg.orientation.x = combined_orientation.x();
    attitude_msg.orientation.y = combined_orientation.y();
    attitude_msg.orientation.z = combined_orientation.z();
    attitude_msg.orientation.w = combined_orientation.w();

    // Publish the message
    attitude_publisher_->publish(attitude_msg);
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
    rclcpp::Subscription<drone_pose_stamped::msg::DronePoseStamped>::SharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<angle_stamped_msg::msg::AngleStamped>::SharedPtr load_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr drone_velocity_subscriber_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr drone_state_subscriber_;
    load_pose_stamped::msg::LoadPoseStamped::SharedPtr load_pose_;
    drone_pose_stamped::msg::DronePoseStamped::SharedPtr drone_pose_;
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
    double pitch;
    double roll;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_publisher_;
    Eigen::Vector3d previous_load_angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d load_angular_acceleration_ = Eigen::Vector3d::Zero();
    rclcpp::Time last_time_ = this->get_clock()->now();  // Initialize with current time
    bool offboard_mode_ = false;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRController>());
    rclcpp::shutdown();
    return 0;
}
