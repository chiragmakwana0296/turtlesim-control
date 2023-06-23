#ifndef ROTATE_CIRCLE_NODE_HPP_
#define ROTATE_CIRCLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <random>

class RotateTurtleNode : public rclcpp::Node
{
public:
    RotateTurtleNode();
    
private:
    void teleportTurtle();
    void posePubTimerCallback();
    void commandPubTimerCallback();
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr rt_real_pose_pub_;
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr rt_noisy_pose_pub_;
    rclcpp::TimerBase::SharedPtr pose_pub_timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    turtlesim::msg::Pose::SharedPtr turtle_pose_;

    std::default_random_engine random_generator_;
    std::normal_distribution<double> noise_distribution_;

    double speed_ = 1.0;    // Speed of the robot (linear velocity)
    double radius_ = 1.0;   // Radius of the circle
};

#endif  // ROTATE_CIRCLE_NODE_HPP_
