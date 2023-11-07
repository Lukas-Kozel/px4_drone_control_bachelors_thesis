#include "DroneVisualWidget.h"

DroneVisualWidget::DroneVisualWidget(rclcpp::Node::SharedPtr node, ConnectionManager *connectionManager, QWidget *parent)
    : QWidget(parent), node(node), connectionManager(connectionManager)
{

    QVBoxLayout *layout = new QVBoxLayout(this);
    topView = new QGraphicsView(this);
    topScene = new QGraphicsScene(this);
    bottomView = new QGraphicsView(this);
    bottomScene = new QGraphicsScene(this);

    topView->setScene(topScene);
    drone1 = topScene->addRect(-60, -5, 120, 10);
    QGraphicsRectItem *motorHolder1 = topScene->addRect(-60, -5, 10, -20);
    QGraphicsRectItem *motorHolder2 = topScene->addRect(50, -5, 10, -20);
    QGraphicsRectItem *motorLeft = topScene->addRect(-75, -25, 40, -5);
    QGraphicsRectItem *motorRight = topScene->addRect(35, -25, 40, -5);
    QPointF droneCenter = drone1->rect().center();
    motorHolder1->setParentItem(drone1);
    motorHolder2->setParentItem(drone1);
    motorLeft->setParentItem(drone1);
    motorRight->setParentItem(drone1);

    QPen axisPen(Qt::red);
    axisPen.setWidth(2);
    DroneVisualWidget::addAxes("x", "-z", drone1, topScene, droneCenter, axisPen);
    load1 = topScene->addEllipse(0, 0, 30, 30);
    line1 = topScene->addLine(0, 5, 0, 0);

    bottomView->setScene(bottomScene);
    drone2 = bottomScene->addRect(-60, -5, 120, 10);
    QGraphicsRectItem *motorHolderBottom1 = bottomScene->addRect(-60, -5, 10, -20);
    QGraphicsRectItem *motorHolderBottom2 = bottomScene->addRect(50, -5, 10, -20);
    QGraphicsRectItem *motorLeftBottom = bottomScene->addRect(-75, -25, 40, -5);
    QGraphicsRectItem *motorRightBottom = bottomScene->addRect(35, -25, 40, -5);
    QPointF droneCenterBottom = drone2->rect().center();
    motorHolderBottom1->setParentItem(drone2);
    motorHolderBottom2->setParentItem(drone2);
    motorLeftBottom->setParentItem(drone2);
    motorRightBottom->setParentItem(drone2);

    axisPen.setColor(Qt::red);
    DroneVisualWidget::addAxes("y", "-z", drone2, bottomScene, droneCenterBottom, axisPen);

    load2 = bottomScene->addEllipse(0, 0, 30, 30);
    line2 = bottomScene->addLine(0, 5, 0, 0);
    layout->addWidget(topView);
    layout->addWidget(bottomView);
    this->setLayout(layout);

    connect(connectionManager, &ConnectionManager::loadPoseReceived, this, &DroneVisualWidget::updateLoadPose);
    connect(connectionManager, &ConnectionManager::connectionStatusChanged, this, &DroneVisualWidget::checkConnectivity);

    // spinning node to update data every 0.1 sec
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, [this]()
            { rclcpp::spin_some(this->node); });
    connect(timer, &QTimer::timeout, this, &DroneVisualWidget::updateGraphs);
    timer->start(100);
}

void DroneVisualWidget::updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr &msg)
{
    loadPoseX = msg->pose.position.x;
    loadPoseY = msg->pose.position.y;
    loadPoseZ = msg->pose.position.z;
}

void DroneVisualWidget::updateGraphs()
{
    if (isConnected)
    {
        loadPoseX = loadPoseX * 400;
        loadPoseZ = loadPoseZ * 400;
        loadPoseY = loadPoseY * 400;

        load1->setPos(loadPoseX - 15, -loadPoseZ - 15);
        line1->setLine(0, 5, loadPoseX, -loadPoseZ);
        load2->setPos(loadPoseY - 15, -loadPoseZ - 15);
        line2->setLine(0, 5, loadPoseY, -loadPoseZ);
    }
    else
    {
        loadPoseX = 0;
        loadPoseZ = -0.35 * 400;
        loadPoseY = 0;
        load1->setPos(loadPoseX - 15, -loadPoseZ - 15);
        line1->setLine(0, 5, loadPoseX, -loadPoseZ);
        load2->setPos(loadPoseY - 15, -loadPoseZ - 15);
        line2->setLine(0, 5, loadPoseY, -loadPoseZ);

        topView->resetTransform();
        topView->centerOn(drone1);
        bottomView->resetTransform();
        bottomView->centerOn(drone2);
    }
}

void DroneVisualWidget::addAxes(std::string axis1, std::string axis2, QGraphicsRectItem *parent, QGraphicsScene *parentScene, QPointF droneCenter, QPen axisPen)
{
    QGraphicsLineItem *xAxisLine = parentScene->addLine(droneCenter.x(), droneCenter.y(), droneCenter.x() + 100, droneCenter.y(), axisPen);
    xAxisLine->setParentItem(parent);

    QPolygonF xArrowHead;
    xArrowHead << QPointF(droneCenter.x() + 100, droneCenter.y())
               << QPointF(droneCenter.x() + 90, droneCenter.y() - 5)
               << QPointF(droneCenter.x() + 90, droneCenter.y() + 5);
    QGraphicsPolygonItem *xArrowItem = parentScene->addPolygon(xArrowHead, axisPen, QBrush(Qt::red));
    xArrowItem->setParentItem(parent);

    axisPen.setColor(Qt::blue);

    QGraphicsLineItem *zAxisLine = parentScene->addLine(droneCenter.x(), droneCenter.y(), droneCenter.x(), droneCenter.y() + 100, axisPen);
    zAxisLine->setParentItem(parent);

    QPolygonF zArrowHead;
    zArrowHead << QPointF(droneCenter.x(), droneCenter.y() + 100)
               << QPointF(droneCenter.x() - 5, droneCenter.y() + 90)
               << QPointF(droneCenter.x() + 5, droneCenter.y() + 90);

    QGraphicsPolygonItem *zArrowItem = parentScene->addPolygon(zArrowHead, axisPen, QBrush(Qt::blue));
    zArrowItem->setParentItem(parent);

    QGraphicsTextItem *xLabel = new QGraphicsTextItem(QString::fromStdString(axis1));

    xLabel->setDefaultTextColor(Qt::red);
    xLabel->setPos(droneCenter.x() + 105 - xLabel->boundingRect().width() / 2, droneCenter.y() - 20);

    QGraphicsTextItem *zLabel = new QGraphicsTextItem(QString::fromStdString(axis2));
    zLabel->setDefaultTextColor(Qt::blue);
    QPointF zArrowTip = QPointF(droneCenter.x(), droneCenter.y() + 100);
    QPointF zLabelPos = zArrowTip - QPointF(zLabel->boundingRect().width(), zLabel->boundingRect().height() / 2);

    zLabel->setPos(zLabelPos);
    parentScene->addItem(zLabel);
    zLabel->setParentItem(parent);
    parentScene->addItem(xLabel);
    xLabel->setParentItem(parent);
}

void DroneVisualWidget::checkConnectivity(bool connected)
{
    isConnected = connected;
}