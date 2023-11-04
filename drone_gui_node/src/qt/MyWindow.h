#pragma once
#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <QMainWindow>
#include "ConnectionManager.h"
#include "DroneVisualWindow.h"
#include <qlayout.h>
#include <QPainter>
#include <qpushbutton.h>
#include <qboxlayout.h>
#include <QLabel>
#include <QTimer>
#include <QWidget>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>
#include <QMenuBar>
#include <QMenu>
#include <QGridLayout>
#include <QAction>
#include <rclcpp/rclcpp.hpp>

class MyWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MyWindow(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent);
    void updateGraph(double drone_velocity_x, double drone_velocity_y, double load_angular_velocity_x, double load_angular_velocity_y);
    ~MyWindow();
    void graphSetup();

private:
    QTimer *timer;
    QGridLayout* gridLayout;
    rclcpp::Node::SharedPtr node;
    QLabel* dronePoseLabel;
    QLabel* loadImuLabel;
    QLabel* loadAngleLabel;
    QLabel* connectionIndicator;
    QLabel* droneVelocityLabel;
    ConnectionManager* connectionManager;
    QtCharts::QBarSet* droneVelocitySetX;
    QtCharts::QBarSet* droneVelocitySetY;
    QtCharts::QBarSet* loadAngularVelocitySetX;
    QtCharts::QBarSet* loadAngularVelocitySetY;
    double drone_velocity_x = 0.0;
    double drone_velocity_y = 0.0;
    double load_angular_velocity_x = 0.0;
    double load_angular_velocity_y = 0.0;

private slots:
    void updateDronePose(const drone_pose_stamped::msg::DronePoseStamped::ConstSharedPtr& msg);
    void updateLoadImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void updateLoadAngle(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr& msg);
    void updateDroneVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
    void updateGraphSlot();
    void updateConnectionIndicator(bool connected);
    void onSwitchToOffboardMode();
    void onDroneOverview();
   // void updateConnectionIndicator(bool connected);
};

#endif // MYWINDOW_H