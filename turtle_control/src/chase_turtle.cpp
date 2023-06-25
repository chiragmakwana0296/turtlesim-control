#include "turtle_control/chase_turtle.hpp"

TurtleChaserNode::TurtleChaserNode()
  : Node("chase_turtle")
{
  pose_sub_ = create_subscription<turtlesim::msg::Pose>(
      "rt_real_pose", 10, std::bind(&TurtleChaserNode::poseCallback, this, std::placeholders::_1));

  action_client_ = rclcpp_action::create_client<turtle_interface::action::GoToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "set_target_pose");
  timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurtleChaserNode::timerCallback, this));

  while (!this->action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Action server not available, waiting...");
  }
}

void TurtleChaserNode::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
    if(!chase_complete){
      std::pair<double, double> pose;
      std::tuple<double, double, double> circle_;
      
      auto goal = turtle_interface::action::GoToPose::Goal();
      goal.x = rt_real_pose_x = pose.first = msg->x;
      goal.y = rt_real_pose_y = pose.second = msg->y;
      
      evidance_pose_.push_back(pose);
      double dist = getDistance(current_x_, current_y_, rt_real_pose_x , rt_real_pose_y);
      // RCLCPP_INFO(this->get_logger(), "Target is at distance of %f units", dist);
      if(dist < 2.0){
        target_reached = true;
        chase_complete = true;
      } 

      if(evidance_pose_.size() == 3 && !reverse_chase){
        circle_ = calculateCircleFromCoordinates(evidance_pose_[0].first, evidance_pose_[0].second, 
                                        evidance_pose_[1].first, evidance_pose_[1].second, 
                                        evidance_pose_[2].first, evidance_pose_[2].second);
        robber_path_ = computeCircleCircumferenceCoordinates(std::get<0>(circle_), std::get<1>(circle_), std::get<2>(circle_), 20);
        evidance_pose_.erase(evidance_pose_.begin());
        std::reverse(robber_path_.begin(), robber_path_.end());
        reverse_chase = true;
      }

      auto send_goal_options = rclcpp_action::Client<turtle_interface::action::GoToPose>::SendGoalOptions();
      send_goal_options.feedback_callback =
            std::bind(&TurtleChaserNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        
      send_goal_options.result_callback =
          std::bind(&TurtleChaserNode::resultCallback, this, std::placeholders::_1);


    if(!target_reached){

      if(use_chase_strategy){
        if(reverse_chase){
          if(index_>=robber_path_.size()){
            index_ = 0;
          }
          goal.x = robber_path_[index_].first;
          goal.x = robber_path_[index_].second;
          this->action_client_->async_send_goal(goal, send_goal_options);
          index_++;
        }
        
      } else{
              RCLCPP_INFO(this->get_logger(), "goal x: %f, y: %f", goal.x, goal.y);
        // send_goal_options.goal_response_callback =
        //   std::bind(&TurtleChaserNode::goalResponseCallback, this, std::placeholders::_1);
        
            this->action_client_->async_send_goal(goal, send_goal_options);
      }

    }else{
      this->action_client_->async_cancel_goal(goal_handle_);
    }
  } else{
      RCLCPP_INFO(this->get_logger(), "Target Reached!  Chase Complete");
  }

}

void TurtleChaserNode::goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtle_interface::action::GoToPose>::SharedPtr> future){
  goal_handle_ = future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}


void TurtleChaserNode::feedbackCallback(
    rclcpp_action::ClientGoalHandle<turtle_interface::action::GoToPose>::SharedPtr goal_handle, 
    const std::shared_ptr<const turtle_interface::action::GoToPose::Feedback> feedback)
{
  goal_handle_ = goal_handle;
  current_x_ = feedback->current_x;
  current_y_ = feedback->current_y;
  // RCLCPP_INFO(this->get_logger(), "Feedback current_x: %f, current_y: %f units", current_x_, current_y_);
}

void TurtleChaserNode::resultCallback(const rclcpp_action::ClientGoalHandle<turtle_interface::action::GoToPose>::WrappedResult & result)
{

}

void TurtleChaserNode::timerCallback()
{


}

double TurtleChaserNode::getDistance(double x1, double y1, double x2, double y2){
    double error_x = x1 - x2;
    double error_y = y1 - y2;
    
    return std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2));
}


 std::vector<std::pair<double, double>> TurtleChaserNode::computeCircleCircumferenceCoordinates(double center_x, double center_y, double radius, int numPoints) {
  double angleIncrement = 2 * M_PI / numPoints;
  std::vector<std::pair<double, double>> points;
  for (int i = 0; i < numPoints; ++i) {
    std::pair<double, double> point;
    double angle = i * angleIncrement;
    point.first = center_x + radius * std::cos(angle);
    point.second = center_y + radius * std::sin(angle);
    points.push_back(point);
  }
  return points;
}


 std::tuple<double, double, double> TurtleChaserNode::calculateCircleFromCoordinates(double x1, double y1, double x2, double y2, double x3, double y3) {
  double center_x, center_y, radius;


  double midpoint1_x = (x1 + x2) / 2;
  double midpoint1_y = (y1 + y2) / 2;

  double midpoint2_x = (x2 + x3) / 2;
  double midpoint2_y = (y2 + y3) / 2;

  double slope1 = -1 / ((y2 - y1) / (x2 - x1));
  double slope2 = -1 / ((y3 - y2) / (x3 - x2));

  center_x = (midpoint2_y - midpoint1_y - slope2 * midpoint2_x + slope1 * midpoint1_x) / (slope1 - slope2);
  center_y = midpoint1_y + slope1 * (center_x - midpoint1_x);
  radius = std::sqrt(std::pow(center_x - x1, 2) + std::pow(center_y - y1, 2));

  return std::make_tuple(center_x, center_y, radius);

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleChaserNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}