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

  // timer_ = this->create_wall_timer(100ms, std::bind(&PIDController::controlCallback, this));

  // PID parameters
  this->declare_parameter<double>("kp", 0.5);
  this->declare_parameter<double>("ki", 0.0);
  this->declare_parameter<double>("kd", 0.1);
  this->declare_parameter<double>("goal_reach_tol", 0.01);
  this->declare_parameter<double>("max_linear_velocity", 1.0);
  this->declare_parameter<double>("max_angular_velocity", 2.0);
  this->declare_parameter<double>("max_linear_acceleration", 0.1);
  this->declare_parameter<double>("max_linear_deceleration", 0.1);

  this->get_parameter("kp", kp_);
  this->get_parameter("ki", ki_);
  this->get_parameter("kd", kd_);
  this->get_parameter("goal_reach_tol", goal_reach_tol_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
    this->get_parameter("max_linear_deceleration", max_linear_deceleration_);

  error_integral_ = 0.0;
  last_error_ = 0.0;

  action_server_ = rclcpp_action::create_server<GoToPose>(
      this,
      "set_target_pose",
      std::bind(&PIDController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PIDController::handleCancel, this, std::placeholders::_1),
      std::bind(&PIDController::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "PID Controller initialized.");

}

rclcpp_action::GoalResponse PIDController::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const GoToPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PIDController::handleCancel(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal cancel request");
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void PIDController::handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&PIDController::execute, this, _1), goal_handle}.detach();
}

void PIDController::execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
  rclcpp::Rate loop_rate(50);
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(this->get_logger(), "Executing goal request: x: %f, y: %f ", goal->x, goal->y);

  // Set the target pose based on the goal request
  auto target_pose = std::make_shared<turtlesim::msg::Pose>();
  target_pose->x = goal->x;
  target_pose->y = goal->y;
  setTargetPose(target_pose);
  
  rclcpp::Time last_time = this->now();

  while (rclcpp::ok())
  {
    if (target_pose_ != nullptr && current_pose_ != nullptr)
    {
      double error = calculateError();

      rclcpp::Time current_time = this->now();  // Get the current time
      double dt = (current_time - last_time).seconds();  // Calculate the time difference
      RCLCPP_INFO(this->get_logger(), "dt: %f", dt);
      auto command = calculateCommand(error, dt);

      publishCommand(command);

      last_time = current_time;  // Update the last time

    }
    if (goal_handle->is_canceling())
    {
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      goal_handle->canceled(std::make_shared<GoToPose::Result>());
      return;
    }

    if (calculateError() < goal_reach_tol_)  
    {
      RCLCPP_INFO(this->get_logger(), "Goal achieved");
      goal_handle->succeed(std::make_shared<GoToPose::Result>());
      return;
    }

    loop_rate.sleep();
  }
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
    ;
    // double error = calculateError();
    // auto command = calculateCommand(error);
    // publishCommand(command);
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

std::pair<double, double> PIDController::calculateCommand(double error, double dt)
{
  error_integral_ += error;
  double error_derivative = error - last_error_;
  last_error_ = error;


  double target_x = target_pose_->x;
  double target_y = target_pose_->y;
  double current_x = current_pose_->x;
  double current_y = current_pose_->y;
  double angle_to_goal = std::atan2(target_y - current_y, target_x - current_x);

  // Acc Decc Profile
  double desired_linear_velocity = kp_ * error + ki_ * error_integral_ + kd_ * error_derivative;
  double current_linear_velocity = current_pose_->linear_velocity; // std::sqrt(std::pow(current_pose_->linear_velocity, 2) + std::pow(current_pose_->angular_velocity, 2));
  double linear_acceleration = (desired_linear_velocity - current_linear_velocity)/dt;

  if (linear_acceleration > max_linear_acceleration_)
  {
    linear_acceleration = max_linear_acceleration_;
  }
  else if (linear_acceleration < -max_linear_deceleration_)
  {
    linear_acceleration = -max_linear_deceleration_;
  }

  double linear_velocity = current_linear_velocity + linear_acceleration * dt;

  // RCLCPP_INFO(this->get_logger(), "linear_velocity: %f", linear_velocity);
  // linear_velocity = kp_ * error + ki_ * error_integral_ + kd_ * error_derivative;
  linear_velocity = std::min(linear_velocity, max_linear_velocity_);

  double angular_velocity = (angle_to_goal - current_pose_->theta);
  angular_velocity = std::max(std::min(angular_velocity, max_angular_velocity_), -max_angular_velocity_);

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

  rclcpp::spin(pid_controller);
  rclcpp::shutdown();
  return 0;
}