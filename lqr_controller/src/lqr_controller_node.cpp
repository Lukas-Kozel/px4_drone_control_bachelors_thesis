#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

class LQRController : public rclcpp::Node
{
public:
    LQRController() : Node("lqr_controller"), control_timer_(nullptr), offboard_timer_(nullptr)
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
        attitude_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_attitude/attitude", 10);
        state_x = Eigen::VectorXd::Zero(4);
        state_y = Eigen::VectorXd::Zero(4);
        K_x_ = (Eigen::MatrixXd(4, 1) << 14.9071, 14.6655, 23.0584, 4.0895).finished();
        K_y_ = (Eigen::MatrixXd(4, 1) << 14.9071, 14.6655, 23.0584, 4.0895).finished();
        drone_mass = 2;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  
            std::bind(&LQRController::control, this)
        );
        offboard_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&LQRController::maintain_offboard_mode, this));
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
        compute_load_angular_acceleration();
    
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

    void compute_load_angular_acceleration(){
        double dt = (this->get_clock()->now() - last_time_).seconds();
        if(dt > 0) {  // Avoid division by zero
            Eigen::Vector3d current_angular_velocity(
                load_imu_->angular_velocity.x,
                load_imu_->angular_velocity.y,
                load_imu_->angular_velocity.z
            );

            // Compute angular acceleration: (current_velocity - previous_velocity) / dt
            load_angular_acceleration_ = (current_angular_velocity - previous_load_angular_velocity_) / dt;

            // Update previous angular velocity and time
            previous_load_angular_velocity_ = current_angular_velocity;
            last_time_ = this->get_clock()->now();
        }
    }

    void create_state_vector(){
        //state_x(0)= load_pose_->position.x + drone_pose_->position.x; //position
        //state_x(1)= drone_velocity_->linear.x;       //velocity
        //state_x(2)= drone_imu_ ->linear_acceleration.x;
        //state_x(3)= load_angle_-> x;
        //state_x(4)= load_imu_->angular_velocity.x;
        //state_x(5)= load_angular_acceleration_(0);

        //state_y(0)= load_pose_->position.y + drone_pose_->position.y; //position
        //state_y(1)= drone_velocity_->linear.y;       //velocity
        //state_y(2)= drone_imu_ ->linear_acceleration.y;
        //state_y(3)= load_angle_-> y;
        //state_y(4)= load_imu_->angular_velocity.y;
        //state_y(5)= load_angular_acceleration_(1);

        state_x(0)= load_pose_->position.x + drone_pose_->position.x; //position
        state_x(1)= drone_velocity_->linear.x;       //velocity
        state_x(2)= load_angle_-> x;
        state_x(3)= load_imu_->angular_velocity.x;

        state_y(0)= load_pose_->position.y + drone_pose_->position.y; //position
        state_y(1)= drone_velocity_->linear.y;       //velocity
        state_y(2)= load_angle_-> y;
        state_y(3)= load_imu_->angular_velocity.y;
    }

    void compute_attitude(){
        double acceleration_x = control_input_x/drone_mass;
        double acceleration_y = control_input_y/drone_mass;
        double acceleration_z = drone_imu_ ->linear_acceleration.z;
        double roll = atan2(-acceleration_x,acceleration_z);
        double pitch = atan2(acceleration_y,acceleration_z);
        publish_control(roll,pitch);
    }

    void control(){
        create_state_vector();
        control_input_x = (-K_x_ * state_x).coeff(0,0);
        control_input_y = (-K_y_ * state_y).coeff(0,0);
        RCLCPP_INFO(this->get_logger(), "Control input x: %.2f",control_input_x);
        RCLCPP_INFO(this->get_logger(), "Control input y: %.2f",control_input_y);
        compute_attitude();

    }

    void publish_control(double roll, double pitch){
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "x500_0"; 
    tf2::Quaternion q;
    q.setRPY(roll, pitch, 0);  // Assuming yaw (theta_z) is 0

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    attitude_publisher_->publish(pose_msg);
   
    }

    void maintain_offboard_mode()
    {
        auto now = this->get_clock()->now();
        auto duration_since_last_offboard_request = now - last_offboard_request_time_;
        
        if (duration_since_last_offboard_request > std::chrono::seconds(2)) {
            set_offboard_mode();
            last_offboard_request_time_ = now;
        }
    }
    void set_offboard_mode()
    {
        auto set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        if (!set_mode_client->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";
        auto future_result = set_mode_client->async_send_request(request);
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
    double control_input_x;
    double control_input_y;
    double drone_mass;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr attitude_publisher_;
    Eigen::Vector3d previous_load_angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d load_angular_acceleration_ = Eigen::Vector3d::Zero();
    rclcpp::Time last_time_ = this->get_clock()->now();  // Initialize with current time
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::Time last_offboard_request_time_ = this->get_clock()->now();  // Initialize with current time

};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRController>());
    rclcpp::shutdown();
    return 0;
}
