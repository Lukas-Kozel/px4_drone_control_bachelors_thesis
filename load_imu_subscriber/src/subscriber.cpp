#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class Subscriber: public rclcpp::Node{
    public:
        Subscriber(): Node("subscriber"){
            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "load_imu",10,std::bind(&Subscriber::topic_callback
                ,this,_1));
        }
    private:
        void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const { //shared pointer is there to automatically alloc and dealloc memory and does not need deep copy data
                RCLCPP_INFO(this->get_logger(), "Orientation: [%f, %f, %f, %f]",
                    msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
                RCLCPP_INFO(this->get_logger(), "Angular Velocity: [%f, %f, %f]",
                    msg->angular_velocity.x, msg->angular_velocity.y,
                    msg->angular_velocity.z);
                RCLCPP_INFO(this->get_logger(), "Linear Acceleration: [%f, %f, %f]",
                    msg->linear_acceleration.x, msg->linear_acceleration.y,
                    msg->linear_acceleration.z);
                }
                rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};


int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}