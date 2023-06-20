#include "turtle_control/pid_pose_controller.hpp"
#include <chrono>

using namespace std::chrono_literals;

PIDController::PIDController() : Node("pid_controller")
{
  target_pose_ = nullptr;
  current_pose_ = nullptr;

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&PIDController::poseCallback, this, std::placeholders::_1));
  

  timer_ = this->create_wall_timer(100ms, std::bind(&PIDController::controlCallback, this));

  // PID parameters
  this->declare_parameter<double>("kp", 0.5);
  this->declare_parameter<double>("ki", 0.0);
  this->declare_parameter<double>("kd", 0.1);

  this->get_parameter("kp", kp_);
  this->get_parameter("ki", ki_);
  this->get_parameter("kd", kd_);

  error_integral_ = 0.0;
  last_error_ = 0.0;

}

void PIDController::setTargetPose(const turtlesim::msg::Pose::SharedPtr target_pose)
{
  target_pose_ = target_pose;
}

void PIDController::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
  current_pose_ = msg;
}

void PIDController::controlCallback()
{
  if (target_pose_ != nullptr && current_pose_ != nullptr)
  {
    double error = calculateError();
    auto command = calculateCommand(error);
    publishCommand(command);
  }
}

double PIDController::calculateError()
{
  double target_x = target_pose_->x;
  double target_y = target_pose_->y;
  double current_x = current_pose_->x;
  double current_y = current_pose_->y;
  double error_x = target_x - current_x;
  double error_y = target_y - current_y;
  double distance_error = std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2));

  return distance_error;
}

std::pair<double, double> PIDController::calculateCommand(double error)
{
  error_integral_ += error;
  double error_derivative = error - last_error_;
  last_error_ = error;

  double linear_velocity = kp_ * error + ki_ * error_integral_ + kd_ * error_derivative;

  double max_linear_velocity = 0.5;
  linear_velocity = std::max(std::min(linear_velocity, max_linear_velocity), -max_linear_velocity);

  double target_x = target_pose_->x;
  double target_y = target_pose_->y;
  double current_x = current_pose_->x;
  double current_y = current_pose_->y;
  double angle_to_goal = std::atan2(target_y - current_y, target_x - current_x);

  double angular_velocity = kp_ * (angle_to_goal - current_pose_->theta);

  double max_angular_velocity = 1.0;
  angular_velocity = std::max(std::min(angular_velocity, max_angular_velocity), -max_angular_velocity);

  return std::make_pair(linear_velocity, angular_velocity);
}

void PIDController::publishCommand(const std::pair<double, double>& command)
{
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = command.first;
  twist.angular.z = command.second;
  cmd_pub_->publish(twist);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto pid_controller = std::make_shared<PIDController>();

  auto target_pose = std::make_shared<turtlesim::msg::Pose>();
  target_pose->x = 2.0;
  target_pose->y = 3.0;
  pid_controller->setTargetPose(target_pose);

  rclcpp::spin(pid_controller);
  rclcpp::shutdown();
  return 0;
}