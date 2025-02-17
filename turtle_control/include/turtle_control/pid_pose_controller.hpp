#ifndef PID_POSE_CONTROLLER_HPP
#define PID_POSE_CONTROLLER_HPP
#include <functional>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtle_interface/action/go_to_pose.hpp"
#include "turtle_interface/srv/rotate_circle.hpp"
#include <cmath>

class PIDController : public rclcpp::Node
{
public:
  using GoToPose = turtle_interface::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

  PIDController();

  void setTargetPose(const turtlesim::msg::Pose::SharedPtr target_pose);
  void setTargetVelocity(const geometry_msgs::msg::Twist::SharedPtr target_pose);

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlCallback();
  double calculatePosError();
  double calculateVelError();
  double mapAngles(double angle);
  std::pair<double, double> controlPose(double error, double dt);
  std::pair<double, double> calculateCommand(double target_linear_vel, double target_angular_vel, double dt);
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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  turtlesim::msg::Pose::SharedPtr target_pose_;
  turtlesim::msg::Pose::SharedPtr current_pose_;
  geometry_msgs::msg::Twist::SharedPtr target_velocity_;

  void handleService(
    const std::shared_ptr<turtle_interface::srv::RotateCircle::Request> request,
    std::shared_ptr<turtle_interface::srv::RotateCircle::Response> response);

  rclcpp::Service<turtle_interface::srv::RotateCircle>::SharedPtr service_;

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

  double error_integral_theta_;
  double last_error_theta_;

  bool got_command_vel_{false};
  bool init_controller_{false};
  bool enable_rotate_circle_{false};
  bool add_acce_deccl_limits_{false};
  double desired_linear_velocity;
  rclcpp::Time last_time;

  std::atomic_bool is_running_; // Atomic flag to control the thread
  std::mutex thread_mutex_; // Mutex to protect access to the thread

};

#endif 
