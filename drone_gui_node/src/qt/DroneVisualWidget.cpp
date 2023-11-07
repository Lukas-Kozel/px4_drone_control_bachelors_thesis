#include "DroneVisualWidget.h"
#include <QVBoxLayout>

DroneVisualWidget::DroneVisualWidget(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent)
    : QWidget(parent), node(node), connectionManager(connectionManager) {
    // Setup the layout
    QVBoxLayout *layout = new QVBoxLayout(this);
    topView = new QGraphicsView(this);
    topScene = new QGraphicsScene(this);
    bottomView = new QGraphicsView(this);
    bottomScene = new QGraphicsScene(this);

    // Setup the top view
    topView->setScene(topScene);
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
    DroneVisualWidget::addAxes("x","-z",droneItem1,topScene,droneCenter,axisPen);
    loadItem1 = topScene->addEllipse(0, 0, 30, 30);  // Initial position of load
    lineItem1 = topScene->addLine(0, 5, 0, 0);  // Initial line connecting drone to load

    // Setup the bottom view
    bottomView->setScene(bottomScene);
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
    DroneVisualWidget::addAxes("y","-z",droneItem2,bottomScene,droneCenterBottom,axisPen);

    loadItem2 = bottomScene->addEllipse(0, 0, 30, 30);  // Initial position of load
    lineItem2 = bottomScene->addLine(0, 5, 0, 0);  // Initial line connecting drone to load
    layout->addWidget(topView);
    layout->addWidget(bottomView);
    this->setLayout(layout);

    
   connect(connectionManager, &ConnectionManager::loadPoseReceived, this, &DroneVisualWidget::updateLoadPose);
   //spinning node to update data every 0.1 sec
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    connect(timer, &QTimer::timeout, this, &DroneVisualWidget::updateGraphs);
    timer->start(100);  // Update every 0.1 second
}

void DroneVisualWidget::updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg)
{
    loadPoseX = msg->pose.position.x;
    loadPoseY = msg->pose.position.y;
    loadPoseZ = msg->pose.position.z;
}

void DroneVisualWidget::updateGraphs()
{
    // Assuming msg contains the x and z coordinates of the load
    loadPoseX = loadPoseX * 400;  // Convert meters to centimeters and scale up by 3
    loadPoseZ = loadPoseZ * 400;  // Convert meters to centimeters and scale up by 3
    loadPoseY = loadPoseY * 400;
    // Update load position and line
    loadItem1->setPos(loadPoseX - 15, -loadPoseZ - 15);  // Adjusted for center of the load (circle)
    lineItem1->setLine(0, 5, loadPoseX, -loadPoseZ);  // Adjusted for center of bottom line of rect and center of the load (circle)
    loadItem2->setPos(loadPoseY - 15, -loadPoseZ - 15);  // Adjusted for center of the load (circle)
    lineItem2->setLine(0, 5, loadPoseY, -loadPoseZ);  // Adjusted for center of bottom line of rect and center of the load (circle)
}

void DroneVisualWidget::addAxes(std::string axis1,std::string axis2, QGraphicsRectItem* parent, QGraphicsScene* parentScene, QPointF droneCenter, QPen axisPen){
    QGraphicsLineItem* xAxisLine = parentScene->addLine(droneCenter.x(), droneCenter.y(), droneCenter.x() + 100, droneCenter.y(), axisPen);
    xAxisLine->setParentItem(parent);

    //  Create the arrowhead for the X axis
    QPolygonF xArrowHead;
    xArrowHead << QPointF(droneCenter.x() + 100, droneCenter.y())
                << QPointF(droneCenter.x() + 90, droneCenter.y() - 5)
                << QPointF(droneCenter.x() + 90, droneCenter.y() + 5);
    QGraphicsPolygonItem* xArrowItem = parentScene->addPolygon(xArrowHead, axisPen, QBrush(Qt::red));
    xArrowItem->setParentItem(parent);

    // Change the pen color for the Z axis
    axisPen.setColor(Qt::blue);

    // Create a line for the Z axis vector
    QGraphicsLineItem* zAxisLine = parentScene->addLine(droneCenter.x(), droneCenter.y(), droneCenter.x(), droneCenter.y() + 100, axisPen);
    zAxisLine->setParentItem(parent);

    // Create the arrowhead for the Z axis
    QPolygonF zArrowHead;
    zArrowHead << QPointF(droneCenter.x(), droneCenter.y() + 100)
                << QPointF(droneCenter.x() - 5, droneCenter.y() + 90)
                << QPointF(droneCenter.x() + 5, droneCenter.y() + 90);

    QGraphicsPolygonItem* zArrowItem = parentScene->addPolygon(zArrowHead, axisPen, QBrush(Qt::blue));
    zArrowItem->setParentItem(parent);

    QGraphicsTextItem* xLabel = new QGraphicsTextItem(QString::fromStdString(axis1));
   
    xLabel->setDefaultTextColor(Qt::red); // Set the text color to red (or any other desired color)
    // Set the position for the label. Adjust the x and y values to position the label correctly relative to the arrowhead
    xLabel->setPos(droneCenter.x() + 105 - xLabel->boundingRect().width()/2, droneCenter.y() - 20);

    QGraphicsTextItem* zLabel = new QGraphicsTextItem(QString::fromStdString(axis2));
    zLabel->setDefaultTextColor(Qt::blue); // Set the text color to blue (or any other desired color)
    // Calculate the position for the label to be to the left of the arrowhead
    QPointF zArrowTip = QPointF(droneCenter.x(), droneCenter.y() + 100);
    QPointF zLabelPos = zArrowTip - QPointF(zLabel->boundingRect().width(), zLabel->boundingRect().height()/2 );

    // Set the calculated position for the label
    zLabel->setPos(zLabelPos);

    // Add the text item to the scene
    parentScene->addItem(zLabel);
    // Set the text item's parent to the droneItem1 so it moves with the drone
    zLabel->setParentItem(parent);

    // Add the text item to the scene
    parentScene->addItem(xLabel);
    // Set the text item's parent to the droneItem1 so it moves with the drone
    xLabel->setParentItem(parent);
}