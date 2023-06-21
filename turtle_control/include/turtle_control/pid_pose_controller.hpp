#ifndef PID_POSE_CONTROLLER_HPP
#define PID_POSE_CONTROLLER_HPP
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtle_interface/action/go_to_pose.hpp"
#include <cmath>

class PIDController : public rclcpp::Node
{
public:
  using GoToPose = turtle_interface::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

  PIDController();

  void setTargetPose(const turtlesim::msg::Pose::SharedPtr target_pose);

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);
  void controlCallback();
  double calculateError();
  std::pair<double, double> calculateCommand(double error, double dt);
  void publishCommand(const std::pair<double, double>& command);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const GoToPose::Goal> goal);
  
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle);

  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp_action::GoalUUID current_goal_id_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  turtlesim::msg::Pose::SharedPtr target_pose_;
  turtlesim::msg::Pose::SharedPtr current_pose_;

  double kp_;
  double ki_;
  double kd_;
  double goal_reach_tol_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double max_linear_acceleration_;
  double max_linear_deceleration_;

  double error_integral_;
  double last_error_;
};

#endif 
