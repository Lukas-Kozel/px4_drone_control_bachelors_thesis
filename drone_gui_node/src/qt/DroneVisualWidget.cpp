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