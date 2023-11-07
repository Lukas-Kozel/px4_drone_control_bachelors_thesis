#include <QHBoxLayout>
#include <QWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageLogger>
#include "ConnectionManager.h"
#include "DroneVisualWidget.h"
#include "MyWindow.h"

MyWindow::MyWindow(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent) : QMainWindow(parent), node(node), connectionManager(connectionManager)
{
    QWidget* centralWidget = new QWidget(this);
    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);  // Main layout to hold left and right layouts
    connectionIndicator = new QLabel(centralWidget);

    if (!node || !connectionManager) {
        qFatal("Null pointer passed to MyWindow constructor");
        return;
    }

    // Setup the graphics view and scene
    DroneVisualWidget *droneVisual = new DroneVisualWidget(node,connectionManager,this);

    QVBoxLayout* leftLayout = new QVBoxLayout();
    QSplitter* leftSplitter = new QSplitter(Qt::Vertical);
    leftSplitter->addWidget(droneVisual);
    leftLayout->addWidget(leftSplitter);
    mainLayout->addLayout(leftLayout, 1);  // Assign more space to the left side

    // Right side layout for buttons and labels
    rightLayout = new QVBoxLayout();
    QPushButton* switchOffboardModeButton = new QPushButton("Switch to Offboard Mode");
    connect(switchOffboardModeButton, &QPushButton::clicked, this, &MyWindow::onSwitchToOffboardMode);
    QPushButton* armButton = new QPushButton("Arm");
    connect(armButton, &QPushButton::clicked, this, &MyWindow::onSwitchToArmedMode);
    rightLayout->addWidget(switchOffboardModeButton);
    rightLayout->addWidget(armButton);

    // Create labels for displaying data
    dronePoseLabel = new QLabel(centralWidget);
    loadImuLabel = new QLabel(centralWidget);
    loadAngleLabel = new QLabel(centralWidget);
    droneVelocityLabel = new QLabel(centralWidget);
    rightLayout->addWidget(dronePoseLabel);
    rightLayout->addWidget(loadImuLabel);
    rightLayout->addWidget(loadAngleLabel);
    rightLayout->addWidget(droneVelocityLabel);

    // Setup the graphs
    graphSetup();
    mainLayout->addLayout(rightLayout, 1);  // Assign equal space to the right side

    QMenuBar* menuBar = setupMenuBar();  // Assuming setupMenuBar is a function to setup the menu bar
    setMenuBar(menuBar);
    setCentralWidget(centralWidget);
    resize(1000, 500);  // Adjusted to give more space

    connect(connectionManager, &ConnectionManager::dronePoseReceived, this, &MyWindow::updateDronePose);
    connect(connectionManager, &ConnectionManager::loadImuReceived, this, &MyWindow::updateLoadImu);
    connect(connectionManager, &ConnectionManager::loadAngleReceived, this, &MyWindow::updateLoadAngle);
    connect(connectionManager, &ConnectionManager::droneVelocityReceived, this, &MyWindow::updateDroneVelocity);
    connect(connectionManager, &ConnectionManager::connectionStatusChanged, this, &MyWindow::updateConnectionIndicator);

   //spinning node to update data every 1 sec
    timer1 = new QTimer(this);
    connect(timer1, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    connect(timer1, &QTimer::timeout, this, &MyWindow::updateGraph);
    connect(timer1,&QTimer::timeout, this, &MyWindow::updateDataTable);
    timer1->start(1000);  // Update every 1 second


}

MyWindow::~MyWindow()
{
    rclcpp::shutdown();
    delete timer1; // Clean up the timer
    delete timer2;
}

void MyWindow::updateDronePose(const drone_pose_stamped::msg::DronePoseStamped::ConstSharedPtr& msg)
{
    drone_pose_x = msg->pose.position.x;
    drone_pose_y = msg->pose.position.y;
    drone_pose_z = msg->pose.position.z;
}

void MyWindow::updateLoadImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    load_angular_velocity_x = msg->angular_velocity.x;
    load_angular_velocity_y = msg->angular_velocity.y;
    load_angular_velocity_z = msg->angular_velocity.z;
}

void MyWindow::updateLoadAngle(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr& msg)
{
    load_angle_x = msg->angle.angle_x;
    load_angle_y = msg->angle.angle_y;
    load_angle_z = msg->angle.angle_z;
}

void MyWindow::updateDroneVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
{
    drone_velocity_x = msg->twist.linear.x;
    drone_velocity_y = msg->twist.linear.y;
    drone_velocity_z = msg->twist.linear.z;
}

void MyWindow::updateGraph() {
    droneVelocitySetX->replace(0, drone_velocity_x);
    droneVelocitySetY->replace(0, drone_velocity_y);
    droneVelocitySetZ->replace(0, drone_velocity_z);
    loadAngularVelocitySetX->replace(0, load_angular_velocity_x);
    loadAngularVelocitySetY->replace(0, load_angular_velocity_y);
    loadAngularVelocitySetZ->replace(0, load_angular_velocity_z);
}

void MyWindow::updateDataTable(){
    std::string position = "Drone position: (" + std::to_string(drone_pose_x) + ", " + std::to_string(drone_pose_y) + ", " + std::to_string(drone_pose_z) + ")";
    dronePoseLabel->setText(QString::fromStdString(position));
    std::string imu = "Load angular velocity: (" + std::to_string(load_angular_velocity_x) + ", " + std::to_string(load_angular_velocity_y) + ", " + std::to_string(load_angular_velocity_z) + ")";
    loadImuLabel->setText(QString::fromStdString(imu));
    std::string load_angle = "Load angle: (" + std::to_string(load_angle_x) + ", " + std::to_string(load_angle_y) + ", " + std::to_string(load_angle_z) + ")";
    loadAngleLabel->setText(QString::fromStdString(load_angle));
    std::string drone_velocity = "Drone velocity: (" + std::to_string(drone_velocity_x) + ", " + std::to_string(drone_velocity_y) + ", " + std::to_string(drone_velocity_z) + ")";
    droneVelocityLabel->setText(QString::fromStdString(drone_velocity));
}

void MyWindow::graphSetup(){
    // Drone velocity graph setup
    droneVelocitySetX = new QtCharts::QBarSet("drone's velocity x");
    droneVelocitySetY = new QtCharts::QBarSet("drone's velocity y");
    droneVelocitySetZ = new QtCharts::QBarSet("drone's velocity z");  // Added for z-axis
    *droneVelocitySetX << 1;  // Initialize with a value
    *droneVelocitySetY << 1;
    *droneVelocitySetZ << 1;
    
    QtCharts::QBarSeries* droneSeries = new QtCharts::QBarSeries();
    droneSeries->append(droneVelocitySetX);
    droneSeries->append(droneVelocitySetY);
    droneSeries->append(droneVelocitySetZ);
    
    QtCharts::QChart* droneChart = new QtCharts::QChart();
    droneChart->addSeries(droneSeries);
    setupAxis(droneChart, droneSeries, "Velocity (m/s)");  // Assuming setupAxis is a helper function to setup axes
    
    droneVelocityGraph = new QtCharts::QChartView(droneChart);
    droneVelocityGraph->setRenderHint(QPainter::Antialiasing);
    
    // Load angular velocity graph setup
    loadAngularVelocitySetX = new QtCharts::QBarSet("load angular velocity x");
    loadAngularVelocitySetY = new QtCharts::QBarSet("load angular velocity y");
    loadAngularVelocitySetZ = new QtCharts::QBarSet("load angular velocity z");  // Added for z-axis
    *loadAngularVelocitySetX << 1;  // Initialize with a value
    *loadAngularVelocitySetY << 1;
    *loadAngularVelocitySetZ << 1;
    
    QtCharts::QBarSeries* loadSeries = new QtCharts::QBarSeries();
    loadSeries->append(loadAngularVelocitySetX);
    loadSeries->append(loadAngularVelocitySetY);
    loadSeries->append(loadAngularVelocitySetZ);
    
    QtCharts::QChart* loadChart = new QtCharts::QChart();
    loadChart->addSeries(loadSeries);
    setupAxis(loadChart, loadSeries, "Angular Velocity (rad/s)");  // Assuming setupAxis is a helper function to setup axes
    
    loadAngularVelocityGraph = new QtCharts::QChartView(loadChart);
    loadAngularVelocityGraph->setRenderHint(QPainter::Antialiasing);
    
    rightLayout->addWidget(droneVelocityGraph);
    rightLayout->addWidget(loadAngularVelocityGraph);
}

void MyWindow::setupAxis(QtCharts::QChart* chart, QtCharts::QBarSeries* series, const QString& yAxisTitle){
    // Common axis setup logic for both graphs
    QtCharts::QValueAxis* axisX = new QtCharts::QValueAxis;
    axisX->setTitleText("Axis (Units)");
    axisX->setGridLineVisible(true); 
    chart->addAxis(axisX, Qt::AlignBottom);
    series->attachAxis(axisX);

    QtCharts::QValueAxis* axisY = new QtCharts::QValueAxis;
    axisY->setTitleText(yAxisTitle);
    axisY->setGridLineVisible(true);
    axisY->setRange(-8, 8);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);
}


void MyWindow::updateConnectionIndicator(bool connected) {
    if (!connectionIndicator) {
        qWarning() << "connectionIndicator is null";
        return;
    }
    QString color = connected ? "green" : "red";
    connectionIndicator->setStyleSheet("background-color: " + color + "; color: white");
    connectionIndicator->setText(connected ? "Connected" : "Disconnected");
}

void MyWindow::onSwitchToOffboardMode()
{
    if(isArmed){
    if (connectionManager->switchToOffboardMode())
    {
        qDebug() << "Successfully switched to offboard mode";
    }
    else
    {
        qDebug() << "Failed to switch to offboard mode";
    }
    }
    else{
        QMessageBox msgBox;
        msgBox.setWindowTitle("Offboard mode issue");
        msgBox.setText("Drone needs to be armed first");
        msgBox.exec();
    }
}

void MyWindow::onSwitchToArmedMode()
{
    if (connectionManager->switchToArmedMode())
    {
        isArmed = true;
        qDebug() << "Successfully switched to armed mode";
    }
    else
    {
        qDebug() << "Failed to switch to armed mode";
        QMessageBox msgBox;
        msgBox.setWindowTitle("Armed mode issue");
        msgBox.setText("something went wrong while arming");
        msgBox.exec();
    }
}

QMenuBar* MyWindow::setupMenuBar()
{
    QMenuBar* menuBar = new QMenuBar;
    QWidget* rightSpacer = new QWidget(this);
    rightSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QHBoxLayout* cornerLayout = new QHBoxLayout(rightSpacer);
    connectionIndicator->setAutoFillBackground(true);
    connectionIndicator->setMinimumSize(120, 30);
    QString color =  "gray";
    QString textColor = "white";
    connectionIndicator->setStyleSheet("background-color: " + color + "; color: " + textColor + ";");
    connectionIndicator->setText("unknown");
    connectionIndicator->setAlignment(Qt::AlignCenter);
    cornerLayout->addWidget(connectionIndicator);  // Add connectionIndicator to the layout
    rightSpacer->setLayout(cornerLayout);
    menuBar->setCornerWidget(rightSpacer, Qt::TopRightCorner);

    return menuBar;
}
