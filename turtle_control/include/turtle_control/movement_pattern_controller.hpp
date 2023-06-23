#ifndef WAYPOINT_FOLLOWER_NODE_HPP_
#define WAYPOINT_FOLLOWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode();

private:
  void sendWaypoints();
  void feedbackCallback(const nav2_msgs::action::FollowWaypoints::Feedback::SharedPtr feedback);
  bool loadWaypointsFromParameter();
  void addWaypoint(const geometry_msgs::msg::PoseStamped& pose);
  void run();

  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t currentWaypoint_;
  rclcpp::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr actionClient_;
};

#endif  // WAYPOINT_FOLLOWER_NODE_HPP_
