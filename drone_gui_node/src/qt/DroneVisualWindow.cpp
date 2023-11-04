#include "DroneVisualWindow.h"

DroneVisualWindow::DroneVisualWindow(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent) : QMainWindow(parent), node(node),connectionManager(connectionManager){
    setWindowTitle("Drone visualization window");
    resize(1000, 1000);
    // Setup the graphics view and scene
    view = new QGraphicsView(this);
    scene = new QGraphicsScene(this);
    view->setScene(scene);

    // Create and add graphics items
    droneItem = scene->addRect(-60, -45, 120, 90);  // Drone rectangle at (0,0)
    loadItem = scene->addEllipse(0, 0, 30, 30);  // Initial position of load
    lineItem = scene->addLine(0, 45, 0, 0);  // Initial line connecting drone to load

    // Layout setup
    QWidget* centralWidget = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(centralWidget);
    layout->addWidget(view);
    centralWidget->setLayout(layout);
    setCentralWidget(centralWidget);

    connect(connectionManager, &ConnectionManager::loadPoseReceived, this, &DroneVisualWindow::updateLoadPose);
}

void DroneVisualWindow::updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg)
{
    // Assuming msg contains the x and z coordinates of the load
    auto x = msg->pose.position.x * 400;  // Convert meters to centimeters and scale up by 3
    auto z = msg->pose.position.z * 400;  // Convert meters to centimeters and scale up by 3

    // Update load position and line
    loadItem->setPos(x - 15, -z - 15);  // Adjusted for center of the load (circle)
    lineItem->setLine(0, 45, x, -z);  // Adjusted for center of bottom line of rect and center of the load (circle)
}