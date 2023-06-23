#include "turtle_control/rotate_circle.hpp"


RotateTurtleNode::RotateTurtleNode()
  : Node("rotate_turtle")
{
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    rt_real_pose_pub_ = create_publisher<turtlesim::msg::Pose>("rt_real_pose", 10);
    rt_noisy_pose_pub_ = create_publisher<turtlesim::msg::Pose>("rt_noisy_pose", 10);
    
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&RotateTurtleNode::poseCallback, this, std::placeholders::_1));

    pose_pub_timer_ = create_wall_timer(std::chrono::milliseconds(5000), std::bind(&RotateTurtleNode::posePubTimerCallback, this));
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&RotateTurtleNode::commandPubTimerCallback, this));
    std::random_device rd;

    random_generator_.seed(rd());
    noise_distribution_ = std::normal_distribution<double>(0.0, 10.0); // Standard deviation of 10

    speed_ = 1.0;    
    radius_ = 1.0; 

    this->declare_parameter<double>("speed", 1.0);
    this->declare_parameter<double>("radius", 0.5);

    this->get_parameter("speed", speed_);
    this->get_parameter("radius", radius_);

    teleportTurtle();
}

void RotateTurtleNode::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
  turtle_pose_ = msg;
}

void RotateTurtleNode::teleportTurtle()
{

  auto teleport_service = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
  while (!teleport_service->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the teleport service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Teleport service not available, waiting...");
  }

  auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
  request->x = 5.54;  
  request->y = 5.54;  
  request->theta = 0; 

  auto future = teleport_service->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call teleport service. Exiting.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Turtle teleported to initial position.");
}

void RotateTurtleNode::commandPubTimerCallback()
{
    this->get_parameter("speed", speed_);
    this->get_parameter("radius", radius_);

    double angular_velocity = speed_ / radius_;

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = speed_;
    twist_msg.angular.z = angular_velocity;

    cmd_pub_->publish(twist_msg);
}

void RotateTurtleNode::posePubTimerCallback()
{
    rt_real_pose_pub_->publish(*turtle_pose_);

    auto pose_msg = turtlesim::msg::Pose();
    pose_msg.x = turtle_pose_->x + noise_distribution_(random_generator_);
    pose_msg.y = turtle_pose_->y + noise_distribution_(random_generator_);
    pose_msg.theta = turtle_pose_->theta + noise_distribution_(random_generator_);
    rt_noisy_pose_pub_->publish(pose_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RotateTurtleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
