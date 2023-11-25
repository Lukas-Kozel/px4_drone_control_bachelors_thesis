#include <QApplication>
#include "MyWindow.h"
#include "ConnectionManager.h"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv)
{
	QApplication app(argc, argv);
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gui_node");
	ConnectionManager connectionManager(node);
    MyWindow window(node, &connectionManager,nullptr);
    window.show();
	return app.exec();
}
