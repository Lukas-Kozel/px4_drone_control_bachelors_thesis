#pragma once

#include <QObject>
#include <QTimer>
#include <QMessageBox>
#include <QDebug>
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include "drone_pose_stamped/msg/drone_pose_stamped.hpp"
#include "load_pose_stamped/msg/load_pose_stamped.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <qwidget.h>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "ConnectionManager.h"


class DroneVisualWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit DroneVisualWindow(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent);
    //~DroneVisualWindow();

private:
    QGraphicsView* view;
    QGraphicsScene* scene;
    rclcpp::Node::SharedPtr node;
    ConnectionManager* connectionManager;
    QGraphicsRectItem* droneItem;
    QGraphicsEllipseItem* loadItem;
    QGraphicsLineItem* lineItem;

private slots:
    void updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg);
};
