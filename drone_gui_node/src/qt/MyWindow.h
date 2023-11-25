#pragma once
#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QSplitter>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QDebug>
#include <fstream>
#include <cmath>
#include <QGraphicsTextItem>
#include <QLabel>
#include <QTimer>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarSeries>
#include <QtCharts/QValueAxis>
#include <QMenuBar>
#include <QWidget>
#include <QSplitter>
#include <QMessageLogger>
#include <QMenu>
#include <rclcpp/rclcpp.hpp>
#include <QFile>
#include <QTextStream>
#include "ConnectionManager.h"
#include "DroneVisualWidget.h"

class MyWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MyWindow(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent = nullptr);
    ~MyWindow();

private:
    rclcpp::Node::SharedPtr node;
    ConnectionManager *connectionManager;
    QTimer* timer1;
    QVBoxLayout* rightLayout;
    QLabel* dronePoseLabel;
    QLabel* loadImuLabel;
    QLabel* loadAngleLabel;
    QLabel* droneVelocityLabel;
    QLabel* connectionIndicator;
    QLabel* stateIndicator;
    QPushButton* controllerButton;
    QPushButton* landingButton;
    QPushButton* armButton;
    QPushButton* takeoffButton;
    QPushButton* turnOffboardModeOffButton;
    QPushButton* switchOffboardModeButton;
    QPushButton* environmentSetupButton;
    QtCharts::QBarSet* droneVelocitySetX;
    QtCharts::QBarSet* droneVelocitySetY;
    QtCharts::QBarSet* droneVelocitySetZ;
    QtCharts::QBarSet* loadAngularVelocitySetX;
    QtCharts::QBarSet* loadAngularVelocitySetY;
    QtCharts::QBarSet* loadAngularVelocitySetZ;
    QtCharts::QChartView* droneVelocityGraph;
    QtCharts::QChartView* loadAngularVelocityGraph;
    double drone_pose_x = 0.0;
    double drone_pose_y = 0.0;
    double drone_pose_z = 0.0;
    double drone_velocity_x = 0.0;
    double drone_velocity_y = 0.0;
    double drone_velocity_z = 0.0;
    double load_angular_velocity_x = 0.0;
    double load_angular_velocity_y = 0.0;
    double load_angular_velocity_z = 0.0;
    double load_angle_x = 0.0;
    double load_angle_y = 0.0;
    double load_angle_z = 0.0;
    bool isArmed = false;
    bool isConnected = false;
    bool offboardMode = false;

private slots:
    void updateDronePose(const drone_pose_stamped::msg::DronePoseStamped::ConstSharedPtr& msg);
    void updateLoadImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void updateLoadAngle(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr& msg);
    void updateDroneVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
    void updateConnectionIndicator(bool connected);
    void updateStateIndicator(std::string mode);
    void onSwitchToOffboardMode();
    void turnOffboardModeOff();
    void onSwitchToArmedMode();
    void onTakeOffMode();
    void onLandingMode();
    void onControllerStart();
    void handleProcessError();
    void onEnvironmentSetup();
    void handleScriptExecutionError();
    void updateGraph();
    void updateDataTable();
    void buttonManager();
    void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void onRestartSimulation();


private:
    void graphSetup();
    void turnOffTheController();
    void setupAxis(QtCharts::QChart* chart, QtCharts::QBarSeries* series, const QString &AxisText, qreal rangeStart, qreal rangeEnd);
    QMenuBar* setupMenuBar();

protected:
    void closeEvent(QCloseEvent *event) override;
};

#endif
