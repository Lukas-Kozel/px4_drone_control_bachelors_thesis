#include <QHBoxLayout>
#include <QWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageLogger>
#include "ConnectionManager.h"
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
    QGraphicsView* topView = new QGraphicsView(this);
    QGraphicsScene* scene = new QGraphicsScene(this);
    topView->setScene(scene);

    // Create and add graphics items
    droneItem = scene->addRect(-60, -45, 120, 90);  // Drone rectangle at (0,0)
    loadItem = scene->addEllipse(0, 0, 30, 30);  // Initial position of load
    lineItem = scene->addLine(0, 45, 0, 0);  // Initial line connecting drone to load


    // Left side layout for DroneVisualWindow
    QVBoxLayout* leftLayout = new QVBoxLayout();
    QSplitter* leftSplitter = new QSplitter(Qt::Vertical);
    QGraphicsView* bottomView = new QGraphicsView();
    leftSplitter->addWidget(topView);
    leftSplitter->addWidget(bottomView);
    leftLayout->addWidget(leftSplitter);
    mainLayout->addLayout(leftLayout, 1);  // Assign more space to the left side

    // Right side layout for buttons and labels
    rightLayout = new QVBoxLayout();
    QPushButton* switchOffboardModeButton = new QPushButton("Switch to Offboard Mode");
    QPushButton* takeOffButton = new QPushButton("Take Off");
    QPushButton* armButton = new QPushButton("Arm");
    rightLayout->addWidget(switchOffboardModeButton);
    rightLayout->addWidget(takeOffButton);
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
    
    connect(connectionManager, &ConnectionManager::loadPoseReceived, this, &MyWindow::updateLoadPose);

   //spinning node to update data every 1 sec
    timer1 = new QTimer(this);
    connect(timer1, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    connect(timer1, &QTimer::timeout, this, &MyWindow::updateGraphSlot);
    timer1->start(1000);  // Update every 1 second

   //spinning node to update data every 0.1 sec
    timer2 = new QTimer(this);
    connect(timer2, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    connect(timer2, &QTimer::timeout, this, &MyWindow::updateLoadPoseGraphX);
    timer2->start(100);  // Update every 0.1 second

}

MyWindow::~MyWindow()
{
    rclcpp::shutdown();
    delete timer1; // Clean up the timer
    delete timer2;
}

void MyWindow::updateDronePose(const drone_pose_stamped::msg::DronePoseStamped::ConstSharedPtr& msg)
{
    std::string position = "Drone position: (" + std::to_string(msg->pose.position.x) + ", " + std::to_string(msg->pose.position.y) + ", " + std::to_string(msg->pose.position.z) + ")";
    dronePoseLabel->setText(QString::fromStdString(position));
}

void MyWindow::updateLoadImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    std::string imu = "Load angular velocity: (" + std::to_string(msg->angular_velocity.x) + ", " + std::to_string(msg->angular_velocity.y) + ", " + std::to_string(msg->angular_velocity.z) + ")";
    loadImuLabel->setText(QString::fromStdString(imu));
    load_angular_velocity_x = msg->angular_velocity.x;
    load_angular_velocity_y = msg->angular_velocity.y;
    load_angular_velocity_z = msg->angular_velocity.z;
}

void MyWindow::updateLoadAngle(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr& msg)
{
    std::string load_angle = "Load angle: (" + std::to_string(msg->angle.angle_x) + ", " + std::to_string(msg->angle.angle_y) + ", " + std::to_string(msg->angle.angle_z) + ")";
    loadAngleLabel->setText(QString::fromStdString(load_angle));
}

void MyWindow::updateDroneVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
{
    std::string drone_velocity = "Drone velocity: (" + std::to_string(msg->twist.linear.x) + ", " + std::to_string(msg->twist.linear.y) + ", " + std::to_string(msg->twist.linear.z) + ")";
    droneVelocityLabel->setText(QString::fromStdString(drone_velocity));
    drone_velocity_x = msg->twist.linear.x;
    drone_velocity_y = msg->twist.linear.y;
    drone_velocity_z = msg->twist.linear.z;
}

void MyWindow::updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg)
{
    loadPoseX = msg->pose.position.x;
    loadPoseY = msg->pose.position.y;
    loadPoseZ = msg->pose.position.z;
}

void MyWindow::updateGraphSlot() {
    updateGraph(drone_velocity_x, drone_velocity_y, drone_velocity_z, load_angular_velocity_x, load_angular_velocity_y,load_angular_velocity_z);
}

void MyWindow::updateGraph(double drone_velocity_x, double drone_velocity_y, double drone_velocity_z, double load_angular_velocity_x, double load_angular_velocity_y, double load_angular_velocity_z) {
    droneVelocitySetX->replace(0, drone_velocity_x);
    droneVelocitySetY->replace(0, drone_velocity_y);
    droneVelocitySetZ->replace(0, drone_velocity_z);
    loadAngularVelocitySetX->replace(0, load_angular_velocity_x);
    loadAngularVelocitySetY->replace(0, load_angular_velocity_y);
    loadAngularVelocitySetZ->replace(0, load_angular_velocity_z);
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
    connectionIndicator->setStyleSheet("background-color: " + color + ";");
    connectionIndicator->setText(connected ? "Connected" : "Disconnected");
}

void MyWindow::onSwitchToOffboardMode()
{
        if (!connectionManager) {
        qWarning() << "connectionIndicator is null";
        return;
    }
    if (connectionManager->switchToOffboardMode())
    {
        qDebug() << "Successfully switched to offboard mode";
    }
    else
    {
        qDebug() << "Failed to switch to offboard mode";
    }
}

QMenuBar* MyWindow::setupMenuBar()
{
    QMenuBar* menuBar = new QMenuBar;
    QWidget* rightSpacer = new QWidget(this);
    rightSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QHBoxLayout* cornerLayout = new QHBoxLayout(rightSpacer);
    connectionIndicator->setAutoFillBackground(true);
    connectionIndicator->setMinimumSize(60, 20);
    QString color =  "gray";
    QString textColor = "white";
    connectionIndicator->setStyleSheet("background-color: " + color + "; color: " + textColor + ";");
    connectionIndicator->setText("unknown");
    cornerLayout->addWidget(connectionIndicator);  // Add connectionIndicator to the layout
    rightSpacer->setLayout(cornerLayout);
    menuBar->setCornerWidget(rightSpacer, Qt::TopRightCorner);

    return menuBar;
}

void MyWindow::updateLoadPoseGraphX(){
    updateGraphX(loadPoseX,loadPoseZ);
}
void MyWindow::updateGraphX(double x, double z)
{
    // Assuming msg contains the x and z coordinates of the load
    x = x * 400;  // Convert meters to centimeters and scale up by 3
    z = z * 400;  // Convert meters to centimeters and scale up by 3

    // Update load position and line
    loadItem->setPos(x - 15, -z - 15);  // Adjusted for center of the load (circle)
    lineItem->setLine(0, 45, x, -z);  // Adjusted for center of bottom line of rect and center of the load (circle)
}