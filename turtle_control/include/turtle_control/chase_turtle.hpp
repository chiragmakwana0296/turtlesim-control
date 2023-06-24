#ifndef CHASE_TURTLE_HPP_
#define CHASE_TURTLE_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtle_interface/action/go_to_pose.hpp" // Replace with your action server header file

class TurtleChaserNode : public rclcpp::Node
{
public:
  TurtleChaserNode();

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);
  void timerCallback();

  void goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtle_interface::action::GoToPose>::SharedPtr> future);
  void feedbackCallback(rclcpp_action::ClientGoalHandle<turtle_interface::action::GoToPose>::SharedPtr goal_handle, const std::shared_ptr<const turtle_interface::action::GoToPose::Feedback> feedback);
  void resultCallback(const rclcpp_action::ClientGoalHandle<turtle_interface::action::GoToPose>::WrappedResult & result);
  double getDistance(double x1, double y1, double x2, double y2);

  double current_x_;
  double current_y_;
  double rt_real_pose_x{99.0};
  double rt_real_pose_y{99.0};
  bool target_reached{false};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp_action::Client<turtle_interface::action::GoToPose>::SharedPtr action_client_;
};

#endif  // CHASE_TURTLE_HPP_
