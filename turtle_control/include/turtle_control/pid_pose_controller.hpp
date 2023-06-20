#ifndef PID_POSE_CONTROLLER_HPP
#define PID_POSE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

class PIDController : public rclcpp::Node
{
public:
  PIDController();

  void setTargetPose(const turtlesim::msg::Pose::SharedPtr target_pose);

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);
  void controlCallback();
  double calculateError();
  std::pair<double, double> calculateCommand(double error);
  void publishCommand(const std::pair<double, double>& command);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  turtlesim::msg::Pose::SharedPtr target_pose_;
  turtlesim::msg::Pose::SharedPtr current_pose_;

  double kp_;
  double ki_;
  double kd_;
  double error_integral_;
  double last_error_;
};

#endif 
