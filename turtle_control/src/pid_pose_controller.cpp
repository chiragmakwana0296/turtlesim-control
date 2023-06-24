#include "turtle_control/pid_pose_controller.hpp"
#include <chrono>

using namespace std::chrono_literals;

PIDController::PIDController() : Node("pid_controller")
{
  target_pose_ = nullptr;
  current_pose_ = nullptr;

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  target_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel/desired", 10, std::bind(&PIDController::velocityCallback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "pose", 10, std::bind(&PIDController::poseCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(50ms, std::bind(&PIDController::controlCallback, this));

  // PID parameters
  this->declare_parameter<double>("kp", 0.5);
  this->declare_parameter<double>("ki", 0.0);
  this->declare_parameter<double>("kd", 0.0);
  this->declare_parameter<double>("goal_reach_tol", 0.01);
  this->declare_parameter<double>("max_linear_velocity", 1.0);
  this->declare_parameter<double>("max_angular_velocity", 2.0);
  this->declare_parameter<double>("max_linear_acceleration", 0.1);
  this->declare_parameter<double>("max_linear_deceleration", 0.1);
  this->declare_parameter<bool>("add_acce_deccl_limits", true);

  this->get_parameter("kp", kp_);
  this->get_parameter("ki", ki_);
  this->get_parameter("kd", kd_);
  this->get_parameter("goal_reach_tol", goal_reach_tol_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
  this->get_parameter("max_linear_deceleration", max_linear_deceleration_);
  this->get_parameter("add_acce_deccl_limits", add_acce_deccl_limits_);

  error_integral_ = 0.0;
  last_error_ = 0.0;

  action_server_ = rclcpp_action::create_server<GoToPose>(
      this,
      "set_target_pose",
      std::bind(&PIDController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PIDController::handleCancel, this, std::placeholders::_1),
      std::bind(&PIDController::handle_accepted, this, std::placeholders::_1));

  service_ = create_service<turtle_interface::srv::RotateCircle>(
    "rotate_circle",
    std::bind(&PIDController::handleService, this, std::placeholders::_1, std::placeholders::_2)
  );
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
  auto result = std::make_shared<GoToPose::Result>();
  auto feedback = std::make_shared<GoToPose::Feedback>();
  double time_taken = 0.0;
  rclcpp::Rate loop_rate(50);
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(this->get_logger(), "Executing goal request: x: %f, y: %f ", goal->x, goal->y);

  // Set the target pose based on the goal request
  auto target_pose = std::make_shared<turtlesim::msg::Pose>();
  target_pose->x = goal->x;
  target_pose->y = goal->y;
  setTargetPose(target_pose);
  
  last_time = this->now();
  while (rclcpp::ok())
  {
    rclcpp::Time current_time = this->now();  // Get the current time
    double dt = (current_time - last_time).seconds();  // Calculate the time difference
    
    if (target_pose_ != nullptr && current_pose_ != nullptr)
    {
      double error_pose = calculatePosError();
      auto command_target = controlPose(error_pose, dt);
      publishCommand(command_target);
      feedback->current_x = current_pose_->x;
      feedback->current_y = current_pose_->y;
      feedback->current_velocity = current_pose_->linear_velocity;
      goal_handle->publish_feedback(feedback);

    }
    if (goal_handle->is_canceling())
    {
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      result->reached = false;
      goal_handle->canceled(result);
      return;
    }

    time_taken += dt;
    if (calculatePosError() < goal_reach_tol_)  
    {      
      result->reached = true;
      result->time_taken = time_taken;
      RCLCPP_INFO(this->get_logger(), "Goal achieved - time_taken: %f", result->time_taken);
      goal_handle->succeed(result);
      return;
    }

    loop_rate.sleep();
    last_time = current_time;  // Update the last time
  }
}

void PIDController::setTargetVelocity(const geometry_msgs::msg::Twist::SharedPtr target_velocity)
{
  target_velocity_ = target_velocity;
}

void PIDController::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{

  setTargetVelocity(msg);
  got_command_vel_ = true;
}

void PIDController::setTargetPose(const turtlesim::msg::Pose::SharedPtr target_pose)
{
  target_pose_ = target_pose;
}

void PIDController::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
  current_pose_ = msg;
}


void PIDController::handleService(
  const std::shared_ptr<turtle_interface::srv::RotateCircle::Request> request,
  std::shared_ptr<turtle_interface::srv::RotateCircle::Response> response)
{
  // Retrieve the center coordinates from the request
  // init_service_req_ = true;
  auto target_pose = std::make_shared<turtlesim::msg::Pose>();
  target_pose->x = request->center_x;
  target_pose->y = request->center_y;
  enable_rotate_circle_ = request->enable;
  setTargetPose(target_pose);

  response->success = true;
}

void PIDController::controlCallback()
{
  if(got_command_vel_){
    if(!init_controller_){
      last_time = this->now();
      init_controller_ = true;
    }
    
    rclcpp::Time current_time = this->now();  // Get the current time
    double dt = (current_time - last_time).seconds();  // Calculate the time difference
    
    if (target_velocity_ != nullptr)
    {
      auto command = calculateCommand(target_velocity_->linear.x, target_velocity_->angular.z, dt);
      publishCommand(command);
    }

    last_time = current_time;  // Update the last time
    // got_command_vel_ = false;
  }
}

double PIDController::calculatePosError()
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

double PIDController::calculateVelError()
{
  double velocity_error = target_velocity_->linear.x - current_pose_->linear_velocity;

  return velocity_error;
}
double PIDController::mapAngles(double angle) {
  if (angle >= 0.0) {
    return angle;
  } else {
    return (angle + 2.0 * M_PI);
  }
}

std::pair<double, double> PIDController::controlPose(double error, double dt)
{
  this->get_parameter("kp", kp_);
  this->get_parameter("ki", ki_);
  this->get_parameter("kd", kd_);
  this->get_parameter("goal_reach_tol", goal_reach_tol_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
  this->get_parameter("max_linear_deceleration", max_linear_deceleration_);
  this->get_parameter("add_acce_deccl_limits", add_acce_deccl_limits_);

  error_integral_ += error;
  double error_derivative = (error - last_error_);
  last_error_ = error;

  double target_x = target_pose_->x;
  double target_y = target_pose_->y;
  double current_x = current_pose_->x;
  double current_y = current_pose_->y;
  double angle_to_goal = std::atan2(target_y - current_y, target_x - current_x);
  double linear_velocity{0.0};
  if(add_acce_deccl_limits_){

    desired_linear_velocity = (kp_ * error + ki_ * error_integral_ + kd_ * error_derivative);
    
    double current_linear_velocity = current_pose_->linear_velocity; //std::sqrt(std::pow(current_pose_->linear_velocity, 2) + std::pow(current_pose_->angular_velocity, 2));
    double linear_acceleration = (desired_linear_velocity - current_linear_velocity)/dt;

    if (linear_acceleration > max_linear_acceleration_)
    {
      linear_acceleration = max_linear_acceleration_;
    }
    else if (linear_acceleration < -max_linear_deceleration_)
    {
      linear_acceleration = -max_linear_deceleration_;
    }
    linear_velocity = current_linear_velocity + (linear_acceleration * dt);
  }else{
      linear_velocity = (kp_ * error + ki_ * error_integral_ + kd_ * error_derivative);
  }

  linear_velocity = std::min(linear_velocity, max_linear_velocity_);

  double angular_velocity =(angle_to_goal - current_pose_->theta);
  
  if (angular_velocity > M_PI) {
    angular_velocity -= angular_velocity;
  }
  if (angular_velocity < -M_PI) {
    angular_velocity += angular_velocity;
  }
  if (!(std::abs(angular_velocity) < M_PI)){
    angular_velocity = max_angular_velocity_ * (angular_velocity > 0 ? max_angular_velocity_ : -max_angular_velocity_);
  }
  angular_velocity = std::max(std::min(angular_velocity, max_angular_velocity_), -max_angular_velocity_);

  return std::make_pair(linear_velocity, angular_velocity);
}


std::pair<double, double> PIDController::calculateCommand(double target_linear_vel, double target_angular_vel, double dt)
{
  this->get_parameter("kp", kp_);
  this->get_parameter("ki", ki_);
  this->get_parameter("kd", kd_);
  this->get_parameter("goal_reach_tol", goal_reach_tol_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
  this->get_parameter("max_linear_deceleration", max_linear_deceleration_);
  this->get_parameter("add_acce_deccl_limits", add_acce_deccl_limits_);

  double linear_velocity{0.0};
  double error = std::abs(target_linear_vel) - current_pose_->linear_velocity;
  error_integral_ += error*dt;
  double error_derivative = (error - last_error_)/dt;
  last_error_ = error;

  // Acc Decc Profile

  if(add_acce_deccl_limits_){
      desired_linear_velocity += (kp_ * error + ki_ * error_integral_ + kd_ * error_derivative);
      double current_linear_velocity = current_pose_->linear_velocity; //std::sqrt(std::pow(current_pose_->linear_velocity, 2) + std::pow(current_pose_->angular_velocity, 2));
      double linear_acceleration = (desired_linear_velocity - current_linear_velocity)/dt;

      if (linear_acceleration > max_linear_acceleration_)
      {
        linear_acceleration = max_linear_acceleration_;
      }
      else if (linear_acceleration < -max_linear_deceleration_)
      {
        linear_acceleration = -max_linear_deceleration_;
      }
      linear_velocity = current_linear_velocity + (linear_acceleration * dt);

  } else{
      desired_linear_velocity += (kp_ * error + ki_ * error_integral_ + kd_ * error_derivative);
      linear_velocity = desired_linear_velocity;
  }
  // RCLCPP_INFO(this->get_logger(), "linear_velocity: %f", linear_velocity);
  linear_velocity = std::min(linear_velocity, max_linear_velocity_);

  double angular_velocity = target_angular_vel;
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