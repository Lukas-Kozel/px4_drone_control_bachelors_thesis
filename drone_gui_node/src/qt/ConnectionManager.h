#pragma once

#include <QObject>
#include <QTimer>
#include <QDebug>
#include <rclcpp/rclcpp.hpp>
#include "drone_pose_stamped/msg/drone_pose_stamped.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <qwidget.h>
#include "angle_stamped_msg/msg/angle_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"


class ConnectionManager : public QObject
{
    Q_OBJECT

public:
    explicit ConnectionManager(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
    void checkForMessages();
    bool switchToOffboardMode();

signals:
    void dronePoseReceived(const drone_pose_stamped::msg::DronePoseStamped::ConstSharedPtr& msg);
    void loadImuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void loadAngleReceived(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr& msg);
    void droneVelocityReceived(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
    void connectionStatusChanged(bool connected);

private:
    QTimer *check_timer_;
    bool drone_pose_received_;
    bool load_imu_received_;
    bool load_angle_received_;
    bool drone_velocity_received_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<drone_pose_stamped::msg::DronePoseStamped>::ConstSharedPtr drone_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr load_imu_subscriber_;
    rclcpp::Subscription<angle_stamped_msg::msg::AngleStamped>::ConstSharedPtr load_angle_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr drone_velocity_subscriber_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;


    void onDronePoseReceived(const drone_pose_stamped::msg::DronePoseStamped::ConstSharedPtr msg);
    void onLoadImuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void onLoadAngleReceived(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr msg);
    void onDroneVelocityReceived(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
    
};
