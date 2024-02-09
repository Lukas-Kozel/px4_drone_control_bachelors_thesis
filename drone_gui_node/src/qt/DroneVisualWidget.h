#ifndef DRONEVISUALWIDGET_H
#define DRONEVISUALWIDGET_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsLineItem>
#include <QVBoxLayout>
#include <QGraphicsTextItem>
#include "ConnectionManager.h"
#include <rclcpp/rclcpp.hpp>
#include <QGraphicsPolygonItem>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


#define radsToDeg(x) (x * 180.0 / M_PI)

class DroneVisualWidget : public QWidget {
    Q_OBJECT

public:
    explicit DroneVisualWidget(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent = nullptr);
    struct DroneVisual {
    QGraphicsRectItem* body;
    QGraphicsRectItem* motorHolder1;
    QGraphicsRectItem* motorHolder2;
    QGraphicsRectItem* motorLeft;
    QGraphicsRectItem* motorRight;
    QPointF center;
    };

private:
    QTimer* timer;
    DroneVisual droneTop;
    DroneVisual droneBottom;
    rclcpp::Node::SharedPtr node;
    ConnectionManager* connectionManager;
    QGraphicsView *topView;
    QGraphicsView *bottomView;
    QGraphicsScene *topScene;
    QGraphicsScene *bottomScene;
    QGraphicsRectItem *drone1;
    QGraphicsRectItem *drone2;
    QGraphicsEllipseItem *load1;
    QGraphicsEllipseItem *load2;
    QGraphicsLineItem *line1;
    QGraphicsLineItem *line2;
    double loadPoseX=0;
    double loadPoseY=0;
    double loadPoseZ=0;
    bool isConnected = false;
    double roll=0;
    double pitch=0;



private slots:
    void updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg);
    void updateOrientation(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void checkConnectivity(bool connected);

private:
    void updateGraphs();
    void updateDronesOrientation();
    void addAxes(std::string axis1,std::string axis2, QGraphicsRectItem* parent, QGraphicsScene* parentScene, QPointF droneCenter, QPen axisPen);



};

#endif
