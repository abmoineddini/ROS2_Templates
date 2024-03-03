#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"

#include <memory>
#include <unistd.h>

using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("rotate_server") {

    srv_ = create_service<Spin>("rotate", std::bind(&ServerNode::rotate_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void rotate_callback(const std::shared_ptr<Spin::Request> request,
                       const std::shared_ptr<Spin::Response> response) {
    /* 
      service server 

      geometry_msgs::msg::Twist message;
      RCLCPP_INFO(this->get_logger(), "I heard a-velocity: '%f'",
                  request->angular_velocity);
      RCLCPP_INFO(this->get_logger(), "I heard direction: '%s'",
                  request->direction.c_str());
      RCLCPP_INFO(this->get_logger(), "I heard time: '%d'", request->time);
  
      if (// condition) {
        response->success = true;
      } 
      else if (// condition) {
        response->success = true;
      } 
      else {
        response->success = false;
      }
    */
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}
