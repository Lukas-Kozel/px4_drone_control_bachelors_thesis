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
    QGraphicsScene* topScene = new QGraphicsScene(this);
    topView->setScene(topScene);

    // Create and add graphics items
    droneItem1 = topScene->addRect(-60, -5, 120, 10); 
    QGraphicsRectItem* motorHolder1 = topScene->addRect(-60, -5, 10, -20);
    QGraphicsRectItem* motorHolder2 = topScene->addRect(50, -5, 10,  -20);
    QGraphicsRectItem* motorLeft = topScene->addRect(-75, -25, 40, -5);
    QGraphicsRectItem* motorRight = topScene->addRect(35, -25, 40, -5);
QPointF droneCenter = droneItem1->rect().center();
motorHolder1->setParentItem(droneItem1);
motorHolder2->setParentItem(droneItem1);
motorLeft->setParentItem(droneItem1);
motorRight->setParentItem(droneItem1);

// Define the pen for the axis lines
QPen axisPen(Qt::red);
axisPen.setWidth(2); // Set the pen width to 2 (or another value for a thicker line)

// Create a line for the X axis vector
QGraphicsLineItem* xAxisLine = topScene->addLine(droneCenter.x(), droneCenter.y(), droneCenter.x() + 100, droneCenter.y(), axisPen);
xAxisLine->setParentItem(droneItem1);

// Create the arrowhead for the X axis
QPolygonF xArrowHead;
xArrowHead << QPointF(droneCenter.x() + 100, droneCenter.y())
            << QPointF(droneCenter.x() + 90, droneCenter.y() - 5)
            << QPointF(droneCenter.x() + 90, droneCenter.y() + 5);
QGraphicsPolygonItem* xArrowItem = topScene->addPolygon(xArrowHead, axisPen, QBrush(Qt::red));
xArrowItem->setParentItem(droneItem1);

// Change the pen color for the Z axis
axisPen.setColor(Qt::blue);

// Create a line for the Z axis vector
QGraphicsLineItem* zAxisLine = topScene->addLine(droneCenter.x(), droneCenter.y(), droneCenter.x(), droneCenter.y() + 100, axisPen);
zAxisLine->setParentItem(droneItem1);

// Create the arrowhead for the Z axis
QPolygonF zArrowHead;
zArrowHead << QPointF(droneCenter.x(), droneCenter.y() + 100)
            << QPointF(droneCenter.x() - 5, droneCenter.y() + 90)
            << QPointF(droneCenter.x() + 5, droneCenter.y() + 90);

QGraphicsPolygonItem* zArrowItem = topScene->addPolygon(zArrowHead, axisPen, QBrush(Qt::blue));
zArrowItem->setParentItem(droneItem1);


    loadItem1 = topScene->addEllipse(0, 0, 30, 30);  // Initial position of load
    lineItem1 = topScene->addLine(0, 5, 0, 0);  // Initial line connecting drone to load

QGraphicsTextItem* xLabel = new QGraphicsTextItem("x");
xLabel->setDefaultTextColor(Qt::red); // Set the text color to red (or any other desired color)
// Set the position for the label. Adjust the x and y values to position the label correctly relative to the arrowhead
xLabel->setPos(droneCenter.x() + 105 - xLabel->boundingRect().width()/2, droneCenter.y() - 20);

QGraphicsTextItem* zLabel = new QGraphicsTextItem("-z");
zLabel->setDefaultTextColor(Qt::blue); // Set the text color to blue (or any other desired color)
// Calculate the position for the label to be to the left of the arrowhead
QPointF zArrowTip = QPointF(droneCenter.x(), droneCenter.y() + 100);
QPointF zLabelPos = zArrowTip - QPointF(zLabel->boundingRect().width(), zLabel->boundingRect().height()/2 );

// Set the calculated position for the label
zLabel->setPos(zLabelPos);

// Add the text item to the scene
topScene->addItem(zLabel);
// Set the text item's parent to the droneItem1 so it moves with the drone
zLabel->setParentItem(droneItem1);

// Add the text item to the scene
topScene->addItem(xLabel);
// Set the text item's parent to the droneItem1 so it moves with the drone
xLabel->setParentItem(droneItem1);



QGraphicsView* bottomView = new QGraphicsView(this);
QGraphicsScene* bottomScene = new QGraphicsScene(this);
bottomView->setScene(bottomScene);

// Create and add graphics items to bottomScene
droneItem2 = bottomScene->addRect(-60, -5, 120, 10);  // Drone rectangle at (0,0)
QGraphicsRectItem* motorHolderBottom1 = bottomScene->addRect(-60, -5, 10, -20);
QGraphicsRectItem* motorHolderBottom2 = bottomScene->addRect(50, -5, 10,  -20);
QGraphicsRectItem* motorLeftBottom = bottomScene->addRect(-75, -25, 40, -5);
QGraphicsRectItem* motorRightBottom = bottomScene->addRect(35, -25, 40, -5);
QPointF droneCenterBottom = droneItem2->rect().center();
motorHolderBottom1->setParentItem(droneItem2);
motorHolderBottom2->setParentItem(droneItem2);
motorLeftBottom->setParentItem(droneItem2);
motorRightBottom->setParentItem(droneItem2);

axisPen.setColor(Qt::red);
// Create a line for the X axis vector
xAxisLine = bottomScene->addLine(droneCenterBottom.x(), droneCenterBottom.y(), droneCenterBottom.x() + 100, droneCenterBottom.y(), axisPen);
xAxisLine->setParentItem(droneItem2);

QPolygonF xArrowHeadBottom;
// Create the arrowhead for the X axis
xArrowHeadBottom << QPointF(droneCenterBottom.x() + 100, droneCenterBottom.y())
            << QPointF(droneCenterBottom.x() + 90, droneCenterBottom.y() - 5)
            << QPointF(droneCenterBottom.x() + 90, droneCenterBottom.y() + 5);
xArrowItem = bottomScene->addPolygon(xArrowHeadBottom, axisPen, QBrush(Qt::red));
xArrowItem->setParentItem(droneItem2);


// Create a line for the Z axis vector with blue color
axisPen.setColor(Qt::blue);
axisPen.setWidth(2);
QGraphicsLineItem* zAxisLineBottom = bottomScene->addLine(droneCenterBottom.x(), droneCenterBottom.y(), droneCenterBottom.x(), droneCenterBottom.y() + 100, axisPen);
zAxisLineBottom->setParentItem(droneItem2);

// Create the arrowhead for the Z axis with blue color
QPolygonF zArrowHeadBottom;
zArrowHeadBottom << QPointF(droneCenterBottom.x(), droneCenterBottom.y() + 100)
                 << QPointF(droneCenterBottom.x() - 5, droneCenterBottom.y() + 90)
                 << QPointF(droneCenterBottom.x() + 5, droneCenterBottom.y() + 90);
QGraphicsPolygonItem* zArrowItemBottom = bottomScene->addPolygon(zArrowHeadBottom, axisPen, QBrush(Qt::blue));
zArrowItemBottom->setParentItem(droneItem2);
    loadItem2 = bottomScene->addEllipse(0, 0, 30, 30);  // Initial position of load
    lineItem2 = bottomScene->addLine(0, 5, 0, 0);  // Initial line connecting drone to load

QGraphicsTextItem* yLabel = new QGraphicsTextItem("y");
yLabel->setDefaultTextColor(Qt::red); // Set the text color to red (or any other desired color)
// Set the position for the label. Adjust the x and y values to position the label correctly relative to the arrowhead
yLabel->setPos(droneCenterBottom.x() + 105 - yLabel->boundingRect().width()/2, droneCenterBottom.y() - 20);

QGraphicsTextItem* zLabelBottom = new QGraphicsTextItem("-z");
zLabelBottom->setDefaultTextColor(Qt::blue); // Set the text color to blue (or any other desired color)
// Calculate the position for the label to be to the left of the arrowhead
QPointF zArrowTipBottom = QPointF(droneCenterBottom.x(), droneCenterBottom.y() + 100);
QPointF zLabelPosBottom = zArrowTipBottom - QPointF(zLabelBottom->boundingRect().width(), zLabelBottom->boundingRect().height()/2 );

zLabelBottom->setPos(zLabelPosBottom);
// Add the text item to the scene
bottomScene->addItem(zLabelBottom);
// Set the text item's parent to the droneItem1 so it moves with the drone
zLabelBottom->setParentItem(droneItem2);

// Add the text item to the scene
bottomScene->addItem(yLabel);
// Set the text item's parent to the droneItem1 so it moves with the drone
yLabel->setParentItem(droneItem2);

    // Left side layout for DroneVisualWindow
    QVBoxLayout* leftLayout = new QVBoxLayout();
    QSplitter* leftSplitter = new QSplitter(Qt::Vertical);
    leftSplitter->addWidget(topView);
    leftSplitter->addWidget(bottomView);
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
    
    connect(connectionManager, &ConnectionManager::loadPoseReceived, this, &MyWindow::updateLoadPose);

   //spinning node to update data every 1 sec
    timer1 = new QTimer(this);
    connect(timer1, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    connect(timer1, &QTimer::timeout, this, &MyWindow::updateGraph);
    connect(timer1,&QTimer::timeout, this, &MyWindow::updateDataTable);
    timer1->start(1000);  // Update every 1 second

   //spinning node to update data every 0.1 sec
    timer2 = new QTimer(this);
    connect(timer2, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    connect(timer2, &QTimer::timeout, this, &MyWindow::updateLoadPoseGraphs);
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

void MyWindow::updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg)
{
    loadPoseX = msg->pose.position.x;
    loadPoseY = msg->pose.position.y;
    loadPoseZ = msg->pose.position.z;
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

void MyWindow::updateLoadPoseGraphs(){
    updateGraphs(loadPoseX,loadPoseZ, loadPoseY);
}
void MyWindow::updateGraphs(double x, double z, double y)
{
    // Assuming msg contains the x and z coordinates of the load
    x = x * 400;  // Convert meters to centimeters and scale up by 3
    z = z * 400;  // Convert meters to centimeters and scale up by 3
    y = y * 400;
    // Update load position and line
    loadItem1->setPos(x - 15, -z - 15);  // Adjusted for center of the load (circle)
    lineItem1->setLine(0, 5, x, -z);  // Adjusted for center of bottom line of rect and center of the load (circle)
    loadItem2->setPos(y - 15, -z - 15);  // Adjusted for center of the load (circle)
    lineItem2->setLine(0, 5, y, -z);  // Adjusted for center of bottom line of rect and center of the load (circle)
}