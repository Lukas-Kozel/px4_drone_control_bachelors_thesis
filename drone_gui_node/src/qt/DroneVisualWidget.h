#ifndef DRONEVISUALWIDGET_H
#define DRONEVISUALWIDGET_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>
#include "ConnectionManager.h"
#include <rclcpp/rclcpp.hpp>
#include <QGraphicsPolygonItem>

class DroneVisualWidget : public QWidget {
    Q_OBJECT

public:
    explicit DroneVisualWidget(rclcpp::Node::SharedPtr node, ConnectionManager* connectionManager, QWidget *parent = nullptr);

private:
    QTimer* timer;
    rclcpp::Node::SharedPtr node;
    ConnectionManager* connectionManager;
    QGraphicsView *topView;
    QGraphicsView *bottomView;
    QGraphicsScene *topScene;
    QGraphicsScene *bottomScene;
    QGraphicsRectItem *droneItem1;
    QGraphicsRectItem *droneItem2;
    QGraphicsEllipseItem *loadItem1;
    QGraphicsEllipseItem *loadItem2;
    QGraphicsLineItem *lineItem1;
    QGraphicsLineItem *lineItem2;
    double loadPoseX=0;
    double loadPoseY=0;
    double loadPoseZ=0;
    bool isConnected = false;


private slots:
    void updateLoadPose(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr& msg);
    void checkConnectivity(bool connected);

private:
    void updateGraphs();
    void addAxes(std::string axis1,std::string axis2, QGraphicsRectItem* parent, QGraphicsScene* parentScene, QPointF droneCenter, QPen axisPen);



};

#endif // DRONEVISUALWIDGET_H
