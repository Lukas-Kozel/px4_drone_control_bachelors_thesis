#include "rclcpp/rclcpp.hpp"

class GuiNode : public rclcpp::Node
{
public:
    GuiNode() : Node("my_ros2_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuiNode>());
    rclcpp::shutdown();
    return 0;
}
