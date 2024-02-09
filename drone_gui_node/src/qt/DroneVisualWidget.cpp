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
    droneTop.body = topScene->addRect(-60, -5, 120, 10);
    droneTop.motorHolder1 = topScene->addRect(-60, -5, 10, -20);
    droneTop.motorHolder2 = topScene->addRect(50, -5, 10, -20);
    droneTop.motorLeft = topScene->addRect(-75, -25, 40, -5);
    droneTop.motorRight = topScene->addRect(35, -25, 40, -5);
    droneTop.center = droneTop.body->rect().center();
    droneTop.motorHolder1->setParentItem(droneTop.body);
    droneTop.motorHolder2->setParentItem(droneTop.body);
    droneTop.motorLeft->setParentItem(droneTop.body);
    droneTop.motorRight->setParentItem(droneTop.body);

    QPen axisPen(Qt::red);
    axisPen.setWidth(2);
    DroneVisualWidget::addAxes("x", "-z", droneTop.body, topScene, droneTop.center, axisPen);
    load1 = topScene->addEllipse(0, 0, 30, 30);
    line1 = topScene->addLine(0, 5, 0, 0);

    bottomView->setScene(bottomScene);

    droneBottom.body = bottomScene->addRect(-60, -5, 120, 10);
    droneBottom.motorHolder1 = bottomScene->addRect(-60, -5, 10, -20);
    droneBottom.motorHolder2 = bottomScene->addRect(50, -5, 10, -20);
    droneBottom.motorLeft = bottomScene->addRect(-75, -25, 40, -5);
    droneBottom.motorRight = bottomScene->addRect(35, -25, 40, -5);
    droneBottom.center = droneTop.body->rect().center();
    droneBottom.motorHolder1->setParentItem(droneBottom.body);
    droneBottom.motorHolder2->setParentItem(droneBottom.body);
    droneBottom.motorLeft->setParentItem(droneBottom.body);
    droneBottom.motorRight->setParentItem(droneBottom.body);

    axisPen.setColor(Qt::red);
    DroneVisualWidget::addAxes("y", "-z", droneBottom.body, bottomScene, droneBottom.center, axisPen);

    load2 = bottomScene->addEllipse(0, 0, 30, 30);
    line2 = bottomScene->addLine(0, 5, 0, 0);
    layout->addWidget(topView);
    layout->addWidget(bottomView);
    this->setLayout(layout);

    connect(connectionManager, &ConnectionManager::dronePoseReceived, this, &DroneVisualWidget::updateOrientation);
    connect(connectionManager, &ConnectionManager::loadPoseReceived, this, &DroneVisualWidget::updateLoadPose);
    connect(connectionManager, &ConnectionManager::connectionStatusChanged, this, &DroneVisualWidget::checkConnectivity);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, [this]()
            { rclcpp::spin_some(this->node); });
    connect(timer, &QTimer::timeout, this, &DroneVisualWidget::updateGraphs);
    connect(timer, &QTimer::timeout, this, &DroneVisualWidget::updateDronesOrientation);
    timer->start(100);
}

void DroneVisualWidget::updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr &msg)
{
    loadPoseX = msg->pose.position.x;
    loadPoseY = msg->pose.position.y;
    loadPoseZ = msg->pose.position.z;
    
}
void DroneVisualWidget::updateOrientation(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg){
    tf2::Quaternion drone_orientation(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    drone_orientation.normalize();
    double load_roll, load_pitch, load_yaw;
    tf2::Matrix3x3(drone_orientation).getRPY(load_roll, load_pitch, load_yaw);
    roll = radsToDeg(load_roll);
    pitch = radsToDeg(load_pitch);
    }

void DroneVisualWidget::updateGraphs()
{
    if (isConnected)
    {
        if(std::abs(loadPoseX) <= 0.35 && std::abs(loadPoseY) <= 0.35 && std::abs(loadPoseZ) <= 0.35){
        loadPoseX = loadPoseX * 400;
        loadPoseZ = loadPoseZ * 400;
        loadPoseY = loadPoseY * 400;

        load1->setPos(loadPoseX - 15, -loadPoseZ - 15);
        line1->setLine(0, 5, loadPoseX, -loadPoseZ);
        load2->setPos(loadPoseY - 15, -loadPoseZ - 15);
        line2->setLine(0, 5, loadPoseY, -loadPoseZ);
        }
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
        topView->centerOn(droneTop.body);
        bottomView->resetTransform();
        bottomView->centerOn(droneBottom.body);
    }
}

void DroneVisualWidget::updateDronesOrientation(){
    // For droneTop, using roll for rotation
    QTransform transformRoll;
    transformRoll.translate(droneTop.center.x(), droneTop.center.y());
    transformRoll.rotate(roll);
    transformRoll.translate(-droneTop.center.x(), -droneTop.center.y());

    droneTop.body->setTransform(transformRoll);
    droneTop.motorHolder1->setTransform(transformRoll);
    droneTop.motorHolder2->setTransform(transformRoll);
    droneTop.motorLeft->setTransform(transformRoll);
    droneTop.motorRight->setTransform(transformRoll);

    // For droneBottom, using pitch for rotation
    QTransform transformPitch;
    transformPitch.translate(droneBottom.center.x(), droneBottom.center.y());
    transformPitch.rotate(pitch);
    transformPitch.translate(-droneBottom.center.x(), -droneBottom.center.y());

    droneBottom.body->setTransform(transformPitch);
    droneBottom.motorHolder1->setTransform(transformPitch);
    droneBottom.motorHolder2->setTransform(transformPitch);
    droneBottom.motorLeft->setTransform(transformPitch);
    droneBottom.motorRight->setTransform(transformPitch);
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