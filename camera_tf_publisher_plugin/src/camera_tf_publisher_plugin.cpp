#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gazebo
{
  class CameraTFPublisherPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/) override
    {
      this->model_ = model;

      if (!rclcpp::ok())
        rclcpp::init(0, nullptr);
      this->node_ = std::make_shared<rclcpp::Node>("camera_tf_publisher");
      this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->node_);

      // Pose of model in world (realsense_rgb_frame)
      ignition::math::Pose3d model_pose = model->WorldPose();

      // Look for the sensor and get its pose relative to the link
      ignition::math::Pose3d sensor_pose = ignition::math::Pose3d::Zero;

      auto sensorManager = gazebo::sensors::SensorManager::Instance();
      auto sensors = sensorManager->GetSensors();
      for (auto& s : sensors)
      {
        if (s->Name().find("realsense_rgb_camera") != std::string::npos)
        {
          sensor_pose = s->Pose();  // pose relative to realsense_rgb_frame
          break;
        }
      }

      // Final pose of camera_depth_optical_frame in world
      this->camera_pose_ = model_pose + sensor_pose;

      // Start timer for TF publish
      this->timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CameraTFPublisherPlugin::PublishTF, this));

      // Spin the node
      this->spin_thread_ = std::thread([this]() {
        rclcpp::spin(this->node_);
      });
    }

    void PublishTF()
    {
      auto now = this->node_->get_clock()->now();

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = "world";
      tf.child_frame_id = "realsense_rgb_camera";
      tf.transform.translation.x = camera_pose_.Pos().X();
      tf.transform.translation.y = camera_pose_.Pos().Y();
      tf.transform.translation.z = camera_pose_.Pos().Z();
      tf.transform.rotation.x = camera_pose_.Rot().X();
      tf.transform.rotation.y = camera_pose_.Rot().Y();
      tf.transform.rotation.z = camera_pose_.Rot().Z();
      tf.transform.rotation.w = camera_pose_.Rot().W();

      broadcaster_->sendTransform(tf);
    //   RCLCPP_INFO(this->node_->get_logger(), "Publishing TF [%s → %s]",
    //           tf.header.frame_id.c_str(),
    //           tf.child_frame_id.c_str());

    //     RCLCPP_INFO(this->node_->get_logger(),
    //                 "  Translation: [%.4f, %.4f, %.4f]",
    //                 tf.transform.translation.x,
    //                 tf.transform.translation.y,
    //                 tf.transform.translation.z);

    //     RCLCPP_INFO(this->node_->get_logger(),
    //                 "  Rotation (quaternion): [%.4f, %.4f, %.4f, %.4f]",
    //                 tf.transform.rotation.x,
    //                 tf.transform.rotation.y,
    //                 tf.transform.rotation.z,
    //                 tf.transform.rotation.w);
    }

  private:
    physics::ModelPtr model_;
    ignition::math::Pose3d camera_pose_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::thread spin_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

  GZ_REGISTER_MODEL_PLUGIN(CameraTFPublisherPlugin)
}
