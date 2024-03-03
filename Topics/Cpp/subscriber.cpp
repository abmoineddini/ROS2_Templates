#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
using namespace std;

class LaserScanSubscriberNode : public rclcpp::Node {
public:
  LaserScanSubscriberNode() : Node("topics_quiz_node") {
    // create simple publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // Create a subscriber to the laserscan topic
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LaserScanSubscriberNode::laserScanCallback, this, std::placeholders::_1));
  }

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    /* 
      create your message and processing here:
      e.g:
      if (msg->ranges.size() > 700) {
        geometry_msgs::msg::Twist message;
  
        
        // Print the range value to the console
        RCLCPP_INFO(get_logger(), "Range at index 0: %f", range_at_0);
        RCLCPP_WARN(get_logger(), "Message does not contain 360 ranges.");
        publisher_->publish(message);
    */
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
