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
    last_execution_time_(this->now())
  {
    using std::placeholders::_1;
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_box_pose", 10, std::bind(&PoseSubscriber::poseCallback, this, _1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
      // Throttle planning to 1 plan every 2 sec
      auto now_time = this->now();
      if ((now_time - last_execution_time_).seconds() < 2.0) {
        return;
      }
      last_execution_time_ = now_time;

      RCLCPP_INFO(this->get_logger(),
        "\n--- Received Pose ---\n"
        "Frame ID: %s\n"
        "Position: [x=%.4f, y=%.4f, z=%.4f]\n"
        "Orientation: [x=%.4f, y=%.4f, z=%.4f, w=%.4f]",
        msg->header.frame_id.c_str(),
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w
      );

      using moveit::planning_interface::MoveGroupInterface;
      MoveGroupInterface move_group_interface(shared_from_this(), "arm");

      // Force base_link frame
      move_group_interface.setPoseReferenceFrame("base_link");

      // Check current planning frame
      RCLCPP_INFO(this->get_logger(), "MoveIt Planning Frame (after change): %s",
                  move_group_interface.getPlanningFrame().c_str());

      // Set the target
      move_group_interface.setPoseTarget(msg->pose);

      // Log what MoveIt will actually use
      auto final_target = move_group_interface.getPoseTarget();
      RCLCPP_INFO(this->get_logger(),
        "Final planning target (frame: %s):\n"
        "  Pos: [%.4f, %.4f, %.4f]\n"
        "  Ori: [%.4f, %.4f, %.4f, %.4f]",
        final_target.header.frame_id.c_str(),
        final_target.pose.position.x, final_target.pose.position.y, final_target.pose.position.z,
        final_target.pose.orientation.x, final_target.pose.orientation.y,
        final_target.pose.orientation.z, final_target.pose.orientation.w
      );

      // Plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto result = move_group_interface.plan(plan);
      bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
        RCLCPP_INFO(this->get_logger(), "✅ Planning successful. Executing...");
        move_group_interface.execute(plan);
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Planning failed!");
      }
  }


  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Time last_execution_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
