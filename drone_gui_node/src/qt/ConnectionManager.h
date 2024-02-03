#pragma once

#include <QObject>
#include <QTimer>
#include <QMessageBox>
#include <QWidget>
#include <QDebug>
#include <rclcpp/rclcpp.hpp>
#include "load_pose_stamped/msg/load_pose_stamped.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "angle_stamped_msg/msg/angle_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/msg/state.hpp"


class ConnectionManager : public QObject
{
    Q_OBJECT

public:
    explicit ConnectionManager(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
    void checkForMessages();
    bool switchToOffboardMode();
    bool switchToArmedMode();
    bool takeOffMode();
    bool droneLanding();
    bool switchTheOffboardModeOff();

signals:
    void dronePoseReceived(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void loadPoseReceived(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg);
    void loadImuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void loadAngleReceived(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr& msg);
    void stateReceived(std::string mode);
    void droneVelocityReceived(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
    void connectionStatusChanged(bool connected);

private:
    QTimer *check_timer_;
    QTimer * drone_visual_timer;
    bool drone_pose_received_;
    bool load_imu_received_;
    bool load_angle_received_;
    bool drone_velocity_received_;
    bool load_pose_received_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<load_pose_stamped::msg::LoadPoseStamped>::ConstSharedPtr load_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr load_imu_subscriber_;
    rclcpp::Subscription<angle_stamped_msg::msg::AngleStamped>::ConstSharedPtr load_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr drone_velocity_subscriber_;
    mavros_msgs::msg::State current_state_;
    rclcpp::Subscription<mavros_msgs::msg::State>::ConstSharedPtr state_subscriber_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr set_armed_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr set_takeoff_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr set_landing_mode_client_;


    void onDronePoseReceived(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    void onLoadImuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void onLoadAngleReceived(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr msg);
    void onDroneVelocityReceived(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
    void onLoadPoseReceived(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr msg);  
    void onStateReceived(const mavros_msgs::msg::State::ConstSharedPtr& msg);  
    
};
