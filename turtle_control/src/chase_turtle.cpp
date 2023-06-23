#include "turtle_control/chase_turtle.hpp"

TurtleChaserNode::TurtleChaserNode()
  : Node("chase_turtle")
{
  pose_sub_ = create_subscription<turtlesim::msg::Pose>(
      "rt_real_pose", 10, std::bind(&TurtleChaserNode::poseCallback, this, std::placeholders::_1));

  action_client_ = rclcpp_action::create_client<turtle_interface::action::GoToPose>(this, "set_target_pose");
  timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurtleChaserNode::timerCallback, this));

  while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Action server not available, waiting...");
  }
}

void TurtleChaserNode::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
  if(!target_reached){
    auto goal = turtle_interface::action::GoToPose::Goal();
    goal.x = rt_real_pose_x = msg->x;
    goal.y = rt_real_pose_y = msg->y;
    
    RCLCPP_INFO(this->get_logger(), "goal x: %f, y: %f", goal.x, goal.y);
    auto send_goal_options = rclcpp_action::Client<turtle_interface::action::GoToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
        std::bind(&TurtleChaserNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

    auto goal_handle = action_client_->async_send_goal(goal, send_goal_options);

  }
}


void TurtleChaserNode::feedbackCallback(rclcpp_action::ClientGoalHandle<turtle_interface::action::GoToPose>::SharedPtr, const std::shared_ptr<const turtle_interface::action::GoToPose::Feedback> feedback)
  {
    current_x = feedback->current_x;
    current_y = feedback->current_y;
  }


void TurtleChaserNode::timerCallback()
{
    double dist = getDistance(current_x, current_y, rt_real_pose_x , rt_real_pose_y);
    RCLCPP_INFO(this->get_logger(), "Target is at distance of %f units", dist);

    if(dist < 3.0){
      RCLCPP_INFO(this->get_logger(), "Target Reached! .. at distance of %f units", dist);
      target_reached = true;
    }
}

double TurtleChaserNode::getDistance(double x1, double y1, double x2, double y2){
    double error_x = x1 - x2;
    double error_y = y1 - y2;
    
    return std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2));
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleChaserNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}