#pragma once
#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <QMainWindow>
#include <QSplitter>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarSeries>
#include <QtCharts/QValueAxis>
#include <QMenuBar>
#include <QMenu>
#include <rclcpp/rclcpp.hpp>
#include "ConnectionManager.h"
#include "DroneVisualWindow.h"

class MyWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MyWindow(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent = nullptr);
    ~MyWindow();

private:
    rclcpp::Node::SharedPtr node;
    ConnectionManager* connectionManager;
    QTimer* timer1;
    QTimer* timer2;
    QVBoxLayout* rightLayout;
    QLabel* dronePoseLabel;
    QLabel* loadImuLabel;
    QLabel* loadAngleLabel;
    QLabel* droneVelocityLabel;
    QLabel* connectionIndicator;
    QtCharts::QBarSet* droneVelocitySetX;
    QtCharts::QBarSet* droneVelocitySetY;
    QtCharts::QBarSet* droneVelocitySetZ;
    QtCharts::QBarSet* loadAngularVelocitySetX;
    QtCharts::QBarSet* loadAngularVelocitySetY;
    QtCharts::QBarSet* loadAngularVelocitySetZ;
    QtCharts::QChartView* droneVelocityGraph;
    QtCharts::QChartView* loadAngularVelocityGraph;
    double drone_velocity_x = 0.0;
    double drone_velocity_y = 0.0;
    double drone_velocity_z = 0.0;
    double load_angular_velocity_x = 0.0;
    double load_angular_velocity_y = 0.0;
    double load_angular_velocity_z = 0.0;
    double loadPoseX = 0.0;
    double loadPoseY = 0.0;
    double loadPoseZ = 0.0;
    QGraphicsEllipseItem* loadItem;
    QGraphicsLineItem* lineItem;
    QGraphicsRectItem*  droneItem;

private slots:
    void updateDronePose(const drone_pose_stamped::msg::DronePoseStamped::ConstSharedPtr& msg);
    void updateLoadImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void updateLoadAngle(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr& msg);
    void updateDroneVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
    void updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg);
    void updateGraphSlot();
    void updateConnectionIndicator(bool connected);
    void onSwitchToOffboardMode();
    void updateLoadPoseGraphX();


private:
    void updateGraph(double drone_velocity_x, double drone_velocity_y, double drone_velocity_z, double load_angular_velocity_x, double load_angular_velocity_y, double load_angular_velocity_z);
    void graphSetup();
    void updateGraphX(double x, double z);
    void setupAxis(QtCharts::QChart* chart, QtCharts::QBarSeries* series, const QString& yAxisTitle);
    QMenuBar* setupMenuBar();
};

#endif // MYWINDOW_H
