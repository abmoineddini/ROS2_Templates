#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "actions_quiz_msg/action/distance.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <thread>

class MyActionServer : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;

  explicit MyActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("distance_as_node", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Distance>(this, "distance_as",
        std::bind(&MyActionServer::handle_goal, this, _1, _2),
        std::bind(&MyActionServer::handle_cancel, this, _1),
        std::bind(&MyActionServer::handle_accepted, this, _1));

    this->subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MyActionServer::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  float variables1_;
  int variables2_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    /* 
      An subscriber example to read value from a topic
    */
  }

  // Set goals
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Distance::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
                goal->seconds);
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Cancel the action if needed
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Accept action from client as soon as possible
  void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  // What the action will excute
  void execute(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // defining messgae definition
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto &message = feedback->current_dist;
    message = 0;
    auto result = std::make_shared<Distance::Result>();

    // 1 hz refresh rate
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) {
      /*
        define what the action will do if excuted
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          result->status = false;
          result->total_dist = distance_;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        // distnace calculation feedback
        distance_ = distance_ + sqrt((x_ - prev_x_) * (x_ - prev_x_) +
                                     (y_ - prev_y_) * (y_ - prev_y_));
        prev_x_ = x_;
        prev_y_ = y_;
        message = distance_;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");
    
        loop_rate.sleep();
      }
    
      // Check if goal is done
      if (rclcpp::ok()) {
        result->status = true;
        result->total_dist = distance_;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    */
    }
  }
}; // class MyActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
