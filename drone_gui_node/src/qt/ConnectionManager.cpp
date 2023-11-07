#include "ConnectionManager.h"

ConnectionManager::ConnectionManager(rclcpp::Node::SharedPtr node, QObject* parent)
    : QObject(parent), node_(node)
{
    drone_pose_received_ = false;
    load_imu_received_= false;
    load_angle_received_= false;
    drone_velocity_received_=false;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    set_armed_mode_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    load_pose_subscriber_ = node_->create_subscription<load_pose_stamped::msg::LoadPoseStamped>(
        "/ros_load_pose", 10, std::bind(&ConnectionManager::onLoadPoseReceived, this, std::placeholders::_1));
    drone_pose_subscriber_ = node_->create_subscription<drone_pose_stamped::msg::DronePoseStamped>(
        "/ros_drone_pose", 10, std::bind(&ConnectionManager::onDronePoseReceived, this, std::placeholders::_1));
    load_imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/load_imu", 10, std::bind(&ConnectionManager::onLoadImuReceived, this, std::placeholders::_1));
    load_angle_subscriber_ = node_->create_subscription<angle_stamped_msg::msg::AngleStamped>(
        "/load_angle", 10, std::bind(&ConnectionManager::onLoadAngleReceived, this, std::placeholders::_1));
    drone_velocity_subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/local_position/velocity_body", qos, std::bind(&ConnectionManager::onDroneVelocityReceived, this, std::placeholders::_1));

    //check for messages every 1s 
    check_timer_ = new QTimer(this);
    connect(check_timer_, &QTimer::timeout, this, &ConnectionManager::checkForMessages);
    check_timer_->start(1000);       
 
}
ConnectionManager::~ConnectionManager(){
    delete check_timer_;
    }

void ConnectionManager::onLoadPoseReceived(const load_pose_stamped::msg::LoadPoseStamped::ConstSharedPtr msg)
{
    load_pose_received_ = true;
    emit loadPoseReceived(msg);
}
void ConnectionManager::onDronePoseReceived(const drone_pose_stamped::msg::DronePoseStamped::ConstSharedPtr msg)
{
    drone_pose_received_ = true;
    emit dronePoseReceived(msg);
}

void ConnectionManager::onLoadImuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    load_imu_received_ = true;
    emit loadImuReceived(msg);
}

void ConnectionManager::onLoadAngleReceived(const angle_stamped_msg::msg::AngleStamped::ConstSharedPtr msg)
{
    load_angle_received_ = true;
    emit loadAngleReceived(msg);
}

void ConnectionManager::onDroneVelocityReceived(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
    drone_velocity_received_ = true;
    emit droneVelocityReceived(msg);
}

void ConnectionManager::checkForMessages()
{
    bool all_received = drone_pose_received_ && load_imu_received_ && load_angle_received_ && drone_velocity_received_ && load_pose_received_;
    if (!all_received) {
        qDebug() << "Not all topics have received messages since the last check.";
    }
    emit connectionStatusChanged(all_received);
    drone_pose_received_ = false;
    load_imu_received_ = false;
    load_angle_received_ = false;
    drone_velocity_received_ = false;
}

bool ConnectionManager::switchToArmedMode() {
    if (!set_armed_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Armed mode issue");
        msgBox.setText("Service \"/mavros/cmd/arming\" is not available");
        msgBox.exec();
        return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;
    auto result_future = set_armed_mode_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result_future) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Armed");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service /mavros/cmd/arming");
        QMessageBox msgBox;
        msgBox.setWindowTitle("Armed mode issue");
        msgBox.setText("Service \"/mavros/cmd/arming\" is not available");
        msgBox.exec();
        return false;
    }
    return true;
}

bool ConnectionManager::switchToOffboardMode() {
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Offboard mode issue");
        msgBox.setText("Service \"/mavros/set_mode\" is not available");
        msgBox.exec();
        return false;
    }

    auto set_mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    set_mode_request->custom_mode = "OFFBOARD";
    auto set_mode_future = set_mode_client_->async_send_request(set_mode_request);

    if (rclcpp::spin_until_future_complete(node_, set_mode_future) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Offboard enabled");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service /mavros/set_mode");
        QMessageBox msgBox;
        msgBox.setWindowTitle("Offboard mode issue");
        msgBox.setText("Service \"/mavros/set_mode\" is not available");
        msgBox.exec();
        return false;
    }
    return true;
}