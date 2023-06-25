#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtle_interface/action/go_to_pose.hpp" 
#include <vector>
#include <utility>

class WaypointFollowerNode : public rclcpp::Node
{
public:
  using GoToWaypoint = turtle_interface::action::GoToPose;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<GoToWaypoint>;

  WaypointFollowerNode()
    : Node("waypoint_follower")
  {
    action_client_ = rclcpp_action::create_client<GoToWaypoint>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "set_target_pose");

    while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the action server.");
        return;
      }
      RCLCPP_INFO(get_logger(), "Action server not available, waiting...");
    }

    waypoints_ = generate_waypoints(); //{{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}};
    for(auto wp: waypoints_){
            RCLCPP_INFO(get_logger(), "Feedback: current_x=%f, current_y=%f", wp.first, wp.second);
    }
    current_waypoint_index_ = 0;
    goal_sent_ = false;

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&WaypointFollowerNode::timerCallback, this));
  }

private:
  rclcpp_action::Client<GoToWaypoint>::SharedPtr action_client_;
  std::vector<std::pair<double, double>> waypoints_;
  size_t current_waypoint_index_;
  bool goal_sent_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timerCallback()
  {
    if (!goal_sent_) {
      sendNextGoal();
      goal_sent_ = true;
    }
  }

  void sendNextGoal()
  {
    if (current_waypoint_index_ < waypoints_.size()) {
      auto goal_msg = GoToWaypoint::Goal();
      goal_msg.x = waypoints_[current_waypoint_index_].first;
      goal_msg.y = waypoints_[current_waypoint_index_].second;

      auto send_goal_options = rclcpp_action::Client<GoToWaypoint>::SendGoalOptions();
      send_goal_options.feedback_callback =
          std::bind(&WaypointFollowerNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback =
          std::bind(&WaypointFollowerNode::resultCallback, this, std::placeholders::_1);

      auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

    } else {
      RCLCPP_INFO(get_logger(), "All goals completed!");
    }
  }

  void feedbackCallback(ClientGoalHandle::SharedPtr, const std::shared_ptr<const GoToWaypoint::Feedback> feedback)
  {
    // RCLCPP_INFO(get_logger(), "Feedback: current_x=%f, current_y=%f", feedback->current_x, feedback->current_y);

  }

  void resultCallback(const ClientGoalHandle::WrappedResult& result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        current_waypoint_index_++;
        goal_sent_ = false;
        RCLCPP_INFO(get_logger(), "Goal succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(get_logger(), "Goal aborted!");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Goal canceled!");
        break;
      default:
        RCLCPP_INFO(get_logger(), "Unknown goal result code: %d", result.code);
        break;
    }
    
  }


std::vector<std::pair<double, double>> generate_waypoints(double spacing_x = 9.0, double spacing_y = 2.0, int num_rows = 5, int num_cols = 2) {
  std::vector<std::pair<double, double>> waypoints;
  
  int direction = 1; //-1;
  double x = 1.0;
  double y = 1.0;
  
  for (int row = 0; row < num_rows; row++) {
    for (int col = 0; col < num_cols; col++) {

      if (direction == 1) {
        x = col * spacing_x + 1.0;
      } else {
        x = (num_cols - col - 1) * spacing_x + 1.0;
      }
      
      y = row * spacing_y + 1.0;
      
      waypoints.push_back(std::make_pair(x, y));
    }

    direction *= -1;
  }
  
  return waypoints;
}

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
