#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <QSlider>
#include <vector>
#include <map>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Needed in the header because rclcpp::Client<> uses complete type
#include <moveit_msgs/srv/get_position_ik.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

// Forward-declare TF classes (do NOT include tf2 headers here)
namespace tf2_ros {
  class Buffer;
  class TransformListener;
}

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // Target buttons
    void on_btnShowJointValues_clicked();
    void on_btnShowPoseValues_clicked();
    void on_btnSaveTarget_clicked();
    void on_btnGoToTarget_clicked();

    // Servo buttons
    void on_btnServoOn_clicked();
    void on_btnServoOff_clicked();

    // Jog buttons
    void onJogButtonPressed();
    void onJogButtonReleased();

    // Speed slider
    void on_sliderSpeed_valueChanged(int value);

    // Program control
    void on_btnStartProgram_clicked();
    void on_btnSopProgram_clicked();

    // Apply joints & move EE
    void on_btnApplyJointPositions_clicked();
    void on_btnMoveToPose_clicked();

private:
    Ui::MainWindow *ui;

    QString kTargetFilePath;

    std::vector<double> current_joint_values;
    std::vector<double> current_pose_values; // x, y, z, roll, pitch, yaw (radians)
    std::map<QString, std::vector<double>> saved_joint_targets;

    struct PoseRPY { double x, y, z, roll, pitch, yaw; };
    std::map<QString, PoseRPY> saved_pose_targets;

    // helpers
    void updateJointLabels();
    void updatePoseLabels();
    void sendJogCommand(double x, double y, double z, double rx, double ry, double rz);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void loadTargetsFromJson();
    void saveTargetsToJson();
    void sendTrajectoryToTarget(const std::vector<double>& target_joints);

    // pose helpers
    bool fetchCurrentPoseRPY(PoseRPY &out);
    bool computeIKToJoints(const PoseRPY &target_pose, std::vector<double> &out_joints);

    // ROS 2 Node and communication
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr jog_publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_on_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_off_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_client_;

    // services
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_pipeline_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_pipeline_client_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;

    // TF (allocated only if headers are available in .cpp)
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // config
    std::string base_frame_ = "base_link";
    std::string ee_link_ = "tool0"; // you set this
    std::vector<std::string> joint_names_ { "J1","J2","J3","J4","J5","J6" };
};

#endif // MAINWINDOW_H
