#include <QHBoxLayout>
#include <QWidget>
#include "ConnectionManager.h"
#include "MyWindow.h"

MyWindow::MyWindow(QWidget *parent) : QMainWindow(parent)
{
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("gui_node");

    QWidget* centralWidget = new QWidget(this);
    gridLayout = new QGridLayout(centralWidget);
    graphSetup();

    connectionIndicator = new QLabel();
    connectionIndicator->setAutoFillBackground(true);
    connectionIndicator->setMinimumSize(60, 20);
    QString color =  "gray";
    QString textColor = "white";
    connectionIndicator->setStyleSheet("background-color: " + color + "; color: " + textColor + ";");
    connectionIndicator->setText("unknown");


    // Create and setup ConnectionManager
    connectionManager = new ConnectionManager(node, this);
    connect(connectionManager, &ConnectionManager::dronePoseReceived, this, &MyWindow::updateDronePose);
    connect(connectionManager, &ConnectionManager::loadImuReceived, this, &MyWindow::updateLoadImu);
    connect(connectionManager, &ConnectionManager::loadAngleReceived, this, &MyWindow::updateLoadAngle);
    connect(connectionManager, &ConnectionManager::droneVelocityReceived, this, &MyWindow::updateDroneVelocity);
    connect(connectionManager, &ConnectionManager::connectionStatusChanged, this, &MyWindow::updateConnectionIndicator);
    
    //initialization of ros2 event loop
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, [this]() {
        rclcpp::spin_some(node);
    });
    connect(timer, &QTimer::timeout, this, &MyWindow::updateGraphSlot);
    timer->start(1000);  // Update every 1 second



    // Create labels for displaying data
    dronePoseLabel = new QLabel(centralWidget);
    loadImuLabel = new QLabel(centralWidget);
    loadAngleLabel = new QLabel(centralWidget);
    droneVelocityLabel = new QLabel(centralWidget);
    gridLayout->addWidget(dronePoseLabel, 0, 0);
    gridLayout->addWidget(loadImuLabel, 1, 0);
    gridLayout->addWidget(loadAngleLabel, 2, 0);
    gridLayout->addWidget(droneVelocityLabel, 3, 0);


    QMenuBar* menuBar = new QMenuBar;
    QMenu* overviewMenu = new QMenu("Drone overview", this);
    QAction* overviewAction = new QAction("open drone overview in new window", this);
    overviewMenu->addAction(overviewAction);
    QMenu* controlMenu = new QMenu("Offboard", this);
    QAction* offboardAction = new QAction("Switch to Offboard Mode", this);
    connect(offboardAction, &QAction::triggered, this, &MyWindow::onSwitchToOffboardMode);
    controlMenu->addAction(offboardAction);
    menuBar->addMenu(controlMenu);

    QWidget* rightSpacer = new QWidget(this);
    rightSpacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QHBoxLayout* cornerLayout = new QHBoxLayout(rightSpacer);
    cornerLayout->addWidget(connectionIndicator);  // Add connectionIndicator to the layout
    rightSpacer->setLayout(cornerLayout);
    menuBar->setCornerWidget(rightSpacer, Qt::TopRightCorner);
    setMenuBar(menuBar);  // This ensures the menu bar stays at the top
    setCentralWidget(centralWidget);
    resize(500, 500);
}

MyWindow::~MyWindow()
{
    rclcpp::shutdown();
    delete timer;  // Clean up the timer
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
}

void MyWindow::updateGraphSlot() {
    updateGraph(drone_velocity_x, drone_velocity_y, load_angular_velocity_x, load_angular_velocity_y);
}

void MyWindow::updateGraph(double drone_velocity_x, double drone_velocity_y, double load_angular_velocity_x, double load_angular_velocity_y) {
    droneVelocitySetX->replace(0, drone_velocity_x);
    droneVelocitySetY->replace(0, drone_velocity_y);
    loadAngularVelocitySetX->replace(0, load_angular_velocity_x);
    loadAngularVelocitySetY->replace(0, load_angular_velocity_y);
}

void MyWindow::graphSetup(){
    droneVelocitySetX = new QtCharts::QBarSet("drone's velocity x");
    droneVelocitySetY = new QtCharts::QBarSet("drone's velocity y");
    *droneVelocitySetX << 1;  // Initialize with two values
    *droneVelocitySetY << 1;
    QtCharts::QBarSeries* series = new QtCharts::QBarSeries();
    series->append(droneVelocitySetX);
    series->append(droneVelocitySetY);
    QtCharts::QChart* chart = new QtCharts::QChart();
    chart->addSeries(series);

    QtCharts::QValueAxis* axisX = new QtCharts::QValueAxis;
    axisX->setTitleText("Axis (Units)");
    axisX->setGridLineVisible(true); 
    chart->addAxis(axisX, Qt::AlignBottom);
    series->attachAxis(axisX);

    QtCharts::QValueAxis* axisY = new QtCharts::QValueAxis;
    axisY->setTitleText("Velocity (m/s)");
    axisY->setGridLineVisible(true);
    axisY->setRange(-8, 8);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);

    QtCharts::QChartView* chartView = new QtCharts::QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    gridLayout->addWidget(chartView, 4, 0, 1, 2);

    loadAngularVelocitySetX = new QtCharts::QBarSet("load angular velocity x");
    loadAngularVelocitySetY = new QtCharts::QBarSet("load angular velocity y");
    *loadAngularVelocitySetX << 1;  // Initialize with a value
    *loadAngularVelocitySetY << 1;
    
    // Create bar series and append the bar sets
    QtCharts::QBarSeries* loadSeries = new QtCharts::QBarSeries();
    loadSeries->append(loadAngularVelocitySetX);
    loadSeries->append(loadAngularVelocitySetY);
    
    // Create a chart, add the series, and customize axes
    QtCharts::QChart* loadChart = new QtCharts::QChart();
    loadChart->addSeries(loadSeries);
    
    QtCharts::QValueAxis* loadAxisX = new QtCharts::QValueAxis;
    loadAxisX->setTitleText("Axis (Units)");
    loadAxisX->setGridLineVisible(true);
    loadChart->addAxis(loadAxisX, Qt::AlignBottom);
    loadSeries->attachAxis(loadAxisX);
    
    QtCharts::QValueAxis* loadAxisY = new QtCharts::QValueAxis;
    loadAxisY->setTitleText("Angular Velocity (rad/s)");
    loadAxisY->setGridLineVisible(true);
    loadAxisY->setRange(-8, 8);
    loadChart->addAxis(loadAxisY, Qt::AlignLeft);
    loadSeries->attachAxis(loadAxisY);
    
    // Create a chart view and add it to the grid layout
    QtCharts::QChartView* loadChartView = new QtCharts::QChartView(loadChart);
    loadChartView->setRenderHint(QPainter::Antialiasing);
    gridLayout->addWidget(loadChartView, 5, 0, 1, 2);
}

void MyWindow::updateConnectionIndicator(bool connected) {
    QString color = connected ? "green" : "red";
    connectionIndicator->setStyleSheet("background-color: " + color + ";");
    connectionIndicator->setText(connected ? "Connected" : "Disconnected");
}

void MyWindow::onSwitchToOffboardMode()
{
    if (connectionManager->switchToOffboardMode())
    {
        qDebug() << "Successfully switched to offboard mode";
    }
    else
    {
        qDebug() << "Failed to switch to offboard mode";
    }
}
