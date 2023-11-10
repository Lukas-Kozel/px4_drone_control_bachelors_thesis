#include "MyWindow.h"

MyWindow::MyWindow(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent) : QMainWindow(parent), node(node), connectionManager(connectionManager)
{

    QWidget* centralWidget = new QWidget(this);
    QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);
    connectionIndicator = new QLabel(centralWidget);

    DroneVisualWidget *droneVisual = new DroneVisualWidget(node,connectionManager,this);

    //left side of the app
    QVBoxLayout* leftLayout = new QVBoxLayout(centralWidget);
    QSplitter* leftSplitter = new QSplitter(Qt::Vertical);
    leftSplitter->addWidget(droneVisual);
    leftLayout->addWidget(leftSplitter);
    mainLayout->addLayout(leftLayout, 1);

    //right side of the app
    rightLayout = new QVBoxLayout(centralWidget);

    switchOffboardModeButton = new QPushButton("Switch to Offboard Mode");
    connect(switchOffboardModeButton, &QPushButton::clicked, this, &MyWindow::onSwitchToOffboardMode);

    turnOffboardModeOffButton = new QPushButton("Turn the Offboard Mode off");
    turnOffboardModeOffButton->setDisabled(true);
    connect(turnOffboardModeOffButton, &QPushButton::clicked, this, &MyWindow::turnOffboardModeOff);

    QPushButton* armButton = new QPushButton("Arm");
    connect(armButton, &QPushButton::clicked, this, &MyWindow::onSwitchToArmedMode);

    QPushButton* takeoffButton = new QPushButton("Takeoff");
    connect(takeoffButton, &QPushButton::clicked, this, &MyWindow::onTakeOffMode);

    landingButton = new QPushButton("Land");
    connect(landingButton, &QPushButton::clicked, this, &MyWindow::onLandingMode);

    controllerButton = new QPushButton("Controller");
    controllerButton->setDisabled(true);
    connect(controllerButton,&QPushButton::clicked, this,&MyWindow::onControllerStart);

    QPushButton* environmentSetupButton = new QPushButton("setup the environment");
    connect(environmentSetupButton,&QPushButton::clicked, this,&MyWindow::onEnvironmentSetup);

    rightLayout->addWidget(environmentSetupButton);
    rightLayout->addWidget(armButton);
    rightLayout->addWidget(takeoffButton);
    rightLayout->addWidget(switchOffboardModeButton);
    rightLayout->addWidget(turnOffboardModeOffButton);
    rightLayout->addWidget(controllerButton);
    rightLayout->addWidget(landingButton);


    //data visualization
    dronePoseLabel = new QLabel(centralWidget);
    loadImuLabel = new QLabel(centralWidget);
    loadAngleLabel = new QLabel(centralWidget);
    droneVelocityLabel = new QLabel(centralWidget);
    rightLayout->addWidget(dronePoseLabel);
    rightLayout->addWidget(loadImuLabel);
    rightLayout->addWidget(loadAngleLabel);
    rightLayout->addWidget(droneVelocityLabel);

    graphSetup();
    mainLayout->addLayout(rightLayout, 1);

    QMenuBar* menuBar = setupMenuBar();
    setMenuBar(menuBar);
    setCentralWidget(centralWidget);
    resize(1500, 750);

    connect(connectionManager, &ConnectionManager::dronePoseReceived, this, &MyWindow::updateDronePose);
    connect(connectionManager, &ConnectionManager::loadImuReceived, this, &MyWindow::updateLoadImu);
    connect(connectionManager, &ConnectionManager::loadAngleReceived, this, &MyWindow::updateLoadAngle);
    connect(connectionManager, &ConnectionManager::droneVelocityReceived, this, &MyWindow::updateDroneVelocity);
    connect(connectionManager, &ConnectionManager::connectionStatusChanged, this, &MyWindow::updateConnectionIndicator);
    connect(connectionManager, &ConnectionManager::connectionStatusChanged, this, &MyWindow::checkConnectivity);

   //spinning node to update data every 1 sec
    timer1 = new QTimer(this);
    connect(timer1, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    connect(timer1, &QTimer::timeout, this, &MyWindow::updateGraph);
    connect(timer1,&QTimer::timeout, this, &MyWindow::updateDataTable);
    timer1->start(1000);

}

MyWindow::~MyWindow()
{
    rclcpp::shutdown();
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
    if(isConnected){
    std::string position = "Drone position: (" + std::to_string(drone_pose_x) + ", " + std::to_string(drone_pose_y) + ", " + std::to_string(drone_pose_z) + ")";
    dronePoseLabel->setText(QString::fromStdString(position));
    std::string imu = "Load angular velocity: (" + std::to_string(load_angular_velocity_x) + ", " + std::to_string(load_angular_velocity_y) + ", " + std::to_string(load_angular_velocity_z) + ")";
    loadImuLabel->setText(QString::fromStdString(imu));
    std::string load_angle = "Load angle: (" + std::to_string(load_angle_x) + ", " + std::to_string(load_angle_y) + ", " + std::to_string(load_angle_z) + ")";
    loadAngleLabel->setText(QString::fromStdString(load_angle));
    std::string drone_velocity = "Drone velocity: (" + std::to_string(drone_velocity_x) + ", " + std::to_string(drone_velocity_y) + ", " + std::to_string(drone_velocity_z) + ")";
    droneVelocityLabel->setText(QString::fromStdString(drone_velocity));
    }
    else{
    dronePoseLabel->setText("Drone position: Data unavailable");
    loadImuLabel->setText("Load angular velocity: Data unavailable");
    loadAngleLabel->setText("Load angle: Data unavailable");
    droneVelocityLabel->setText("Drone velocity: Data unavailable");
    }
}

void MyWindow::graphSetup(){
    droneVelocitySetX = new QtCharts::QBarSet("drone's velocity x");
    droneVelocitySetY = new QtCharts::QBarSet("drone's velocity y");
    droneVelocitySetZ = new QtCharts::QBarSet("drone's velocity z");
    //init with some default value
    *droneVelocitySetX << 1;
    *droneVelocitySetY << 1;
    *droneVelocitySetZ << 1;
    
    QtCharts::QBarSeries* droneSeries = new QtCharts::QBarSeries();
    droneSeries->append(droneVelocitySetX);
    droneSeries->append(droneVelocitySetY);
    droneSeries->append(droneVelocitySetZ);
    
    QtCharts::QChart* droneChart = new QtCharts::QChart();
    droneChart->addSeries(droneSeries);
    setupAxis(droneChart, droneSeries, "Velocity (m/s)",-10,10);
    
    droneVelocityGraph = new QtCharts::QChartView(droneChart);
    droneVelocityGraph->setRenderHint(QPainter::Antialiasing);
    
    loadAngularVelocitySetX = new QtCharts::QBarSet("load's angular velocity x");
    loadAngularVelocitySetY = new QtCharts::QBarSet("load's angular velocity y");
    loadAngularVelocitySetZ = new QtCharts::QBarSet("load's angular velocity z");
    //init with some default value
    *loadAngularVelocitySetX << 1;
    *loadAngularVelocitySetY << 1;
    *loadAngularVelocitySetZ << 1;
    
    QtCharts::QBarSeries* loadSeries = new QtCharts::QBarSeries();
    loadSeries->append(loadAngularVelocitySetX);
    loadSeries->append(loadAngularVelocitySetY);
    loadSeries->append(loadAngularVelocitySetZ);
    
    QtCharts::QChart* loadChart = new QtCharts::QChart();
    loadChart->addSeries(loadSeries);
    setupAxis(loadChart, loadSeries, "Angular Velocity (rad/s)",-3,3);
    
    loadAngularVelocityGraph = new QtCharts::QChartView(loadChart);
    loadAngularVelocityGraph->setRenderHint(QPainter::Antialiasing);
    
    rightLayout->addWidget(droneVelocityGraph);
    rightLayout->addWidget(loadAngularVelocityGraph);
}

void MyWindow::setupAxis(QtCharts::QChart* chart, QtCharts::QBarSeries* series, const QString &AxisText, qreal rangeStart, qreal rangeEnd){
    QtCharts::QValueAxis* axisX = new QtCharts::QValueAxis;
    axisX->setGridLineVisible(true); 
    chart->addAxis(axisX, Qt::AlignBottom);
    series->attachAxis(axisX);

    QtCharts::QValueAxis* axisY = new QtCharts::QValueAxis;
    axisY->setTitleText(AxisText);
    axisY->setGridLineVisible(true);
    axisY->setRange(rangeStart, rangeEnd);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);
}

void MyWindow::updateConnectionIndicator(bool connected) {
    QString color = connected ? "green" : "red";
    connectionIndicator->setStyleSheet("background-color: " + color + "; color: white");
    connectionIndicator->setText(connected ? "Connected" : "Disconnected");
}

void MyWindow::onSwitchToOffboardMode()
{
    if(isArmed){
    if (connectionManager->switchToOffboardMode())
    {
        controllerButton->setDisabled(false);
        switchOffboardModeButton->setDisabled(true);
        turnOffboardModeOffButton->setDisabled(false);
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

void MyWindow::turnOffboardModeOff(){
    if (connectionManager->switchTheOffboardModeOff())
    {
        controllerButton->setDisabled(true);
        switchOffboardModeButton->setDisabled(false);
        turnOffboardModeOffButton->setDisabled(true);
        qDebug() << "Successfully switched off the offboard mode";
    }
    else
    {
        qDebug() << "Failed to switch off the offboard mode";
    }
}

void MyWindow::onSwitchToArmedMode()
{
    if (connectionManager->switchToArmedMode())
    {
        isArmed = true;
        landingButton->setDisabled(false);
        qDebug() << "Successfully switched to armed mode";
    }
    else
    {
        qDebug() << "Failed to switch to armed mode";
    }
}

QMenuBar* MyWindow::setupMenuBar()
{
    QMenuBar* menuBar = new QMenuBar(this);
    QWidget* rightSpacer = new QWidget(menuBar);
    rightSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QHBoxLayout* cornerLayout = new QHBoxLayout(rightSpacer);
    connectionIndicator->setAutoFillBackground(true);
    connectionIndicator->setMinimumSize(120, 30);
    connectionIndicator->setStyleSheet("background-color: gray; color: white;");
    connectionIndicator->setText("unknown");
    connectionIndicator->setAlignment(Qt::AlignCenter);
    cornerLayout->addWidget(connectionIndicator);
    rightSpacer->setLayout(cornerLayout);
    menuBar->setCornerWidget(rightSpacer, Qt::TopRightCorner);

    return menuBar;
}
void MyWindow::checkConnectivity(bool connected){
    isConnected = connected;
}

void MyWindow::onControllerStart(){
    QProcess *process = new QProcess(this);
    QStringList arguments;
    process->start("ros2", arguments << "run" << "lqr_controller" << "lqr_controller_node");
    connect(process, &QProcess::errorOccurred, this, &MyWindow::handleProcessError);
}

void MyWindow::handleProcessError(){
    QMessageBox msgBox;
    msgBox.setWindowTitle("Controller node issue");
    msgBox.setText("controller node can not be loaded");
    msgBox.exec();
}

void MyWindow::onEnvironmentSetup(){
    QProcess *process = new QProcess(this);
    process->setWorkingDirectory("/home/luky/mavros_ros2_ws/src/scripts");
    process->start("/bin/bash",QStringList() << "./setup_ws.sh");
    connect(process, &QProcess::errorOccurred, this, &MyWindow::handleScriptExecutionError);
}


void MyWindow::handleScriptExecutionError(){
    QMessageBox msgBox;
    msgBox.setWindowTitle("Environment setup issue");
    msgBox.setText("script could not be executed");
    msgBox.exec();
}

void MyWindow::onTakeOffMode(){
    if (connectionManager->takeOffMode())
    {
        isArmed = true;
        qDebug() << "Successfully switched to takeoff mode";
    }
    else
    {
        qDebug() << "Failed to switch to takeoff mode";
    }   
}

void MyWindow::onLandingMode(){
    if (connectionManager->droneLanding())
    {
        controllerButton->setDisabled(true);
        landingButton->setDisabled(true);
        qDebug() << "Successfully switched to landing mode";
    }
    else
    {
        qDebug() << "Failed to switch to landing mode";
    } 
}