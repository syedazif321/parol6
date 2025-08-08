// move_to_box_once.cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <chrono>

class PoseSubscriber : public rclcpp::Node
{
public:
  PoseSubscriber()
  : Node("move_to_detected_box"),
    has_executed_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Intialized 'move_to_detected_box' node.");
    RCLCPP_INFO(this->get_logger(), "Waiting for the first /detected_box_pose...");

    // Create subscription
    using std::placeholders::_1;
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_box_pose",
      10,
      std::bind(&PoseSubscriber::poseCallback, this, _1)
    );
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Prevent multiple executions
    if (has_executed_) {
      RCLCPP_WARN(this->get_logger(), "Already executed. Ignoring new detection.");
      return;
    }

    has_executed_ = true;

    // RCLCPP_INFO(this->get_logger(),
    //   "Frame ID: %s\n"
    //   "Position [x, y, z]: [%.4f, %.4f, %.4f]\n"
    //   "Orientation [x, y, z, w]: [%.4f, %.4f, %.4f, %.4f]",
    //   msg->header.frame_id.c_str(),
    //   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
    //   msg->pose.orientation.x, msg->pose.orientation.y,
    //   msg->pose.orientation.z, msg->pose.orientation.w
    // );

    pose_sub_.reset();
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm" 
    );

    move_group->setPoseReferenceFrame("base_link");
    RCLCPP_INFO(this->get_logger(), "MoveIt planning frame: %s", move_group->getPlanningFrame().c_str());
    move_group->setPoseTarget(*msg);


    auto target = move_group->getPoseTarget();
    RCLCPP_INFO(this->get_logger(), "Target set. Planning to: [%.3f, %.3f, %.3f]", 
                target.pose.position.x, target.pose.position.y, target.pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful! Executing motion...");
      move_group->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Robot reached the target. Motion complete.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed! Check pose feasibility or collision.");
    }


    std::this_thread::sleep_for(std::chrono::seconds(2)); 
    rclcpp::shutdown();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  bool has_executed_;
};

int main(int argc, char* argv[])
{

  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting one-shot box mover...");
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}