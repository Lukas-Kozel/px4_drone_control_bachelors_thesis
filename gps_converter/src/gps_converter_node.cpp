#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath> // for cos(), sin()

class NavSatToCartesian : public rclcpp::Node
{
public:
  NavSatToCartesian()
  : Node("gps_converter_node")
  {
    cartesian_pub_ = this->create_publisher<geometry_msgs::msg::Point>("load_coordinates", 10);
    navsat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "load_gps", 10, std::bind(&NavSatToCartesian::navsatCallback, this, std::placeholders::_1));
  }

private:
  void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // These should be set to whatever the GPS coordinates are at the Gazebo world origin
    double lat0 = 47.397743; // latitude of Gazebo origin in degrees
    double lon0 = 8.545594; // longitude of Gazebo origin in degrees
    double alt0 = 0.0; // altitude of Gazebo origin in meters

    double dLat = msg->latitude - lat0;
    double dLon = msg->longitude - lon0;
    double dAlt = msg->altitude - alt0;

    // Approximate conversion (assuming small distances and ignoring Earth's curvature)
    const double R = 6371000.0; // Earth's radius in meters
    double x = dLon * R * cos(lat0 * M_PI / 180.0);
    double y = dLat * R;
    double z = dAlt;

    // Publish the Cartesian coordinates
    auto cartesian_msg = geometry_msgs::msg::Point();
    cartesian_msg.x = x;
    cartesian_msg.y = y;
    cartesian_msg.z = z;
    cartesian_pub_->publish(cartesian_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cartesian_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavSatToCartesian>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
