#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PickAndPlaceNode : public rclcpp::Node
{
public:
  PickAndPlaceNode() : Node("pick_and_place_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(this->get_logger(), "[PickAndPlaceNode] Waiting for robot_description...");

    // Wait until robot_description is available
    while (!this->has_parameter("robot_description")) {
      RCLCPP_WARN(this->get_logger(), "Waiting for robot_description...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Initialize MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.01);

    // Subscribe to detected poses
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_box_pose", 10,
      std::bind(&PickAndPlaceNode::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[PickAndPlaceNode] Ready. Listening to /detected_box_pose...");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose, planning to target...");

    move_group_->setPoseTarget(*msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Plan successful. Executing...");
      move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickAndPlaceNode>());
  rclcpp::shutdown();
  return 0;
}
