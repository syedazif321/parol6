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

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

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

private:
    Ui::MainWindow *ui;

    QString kTargetFilePath;

    std::vector<double> current_joint_values;
    std::vector<double> current_pose_values; // x, y, z, roll, pitch, yaw
    std::map<QString, std::vector<double>> saved_joint_targets;
    std::map<QString, std::vector<double>> saved_pose_targets;

    void updateJointLabels();
    void updatePoseLabels();
    void sendJogCommand(double x, double y, double z, double rx, double ry, double rz);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void loadTargetsFromJson();
    void saveTargetsToJson();
    void sendTrajectoryToTarget(const std::vector<double>& target_joints);

    // ROS 2 Node and communication
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr jog_publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_on_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_off_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_client_;
};

#endif // MAINWINDOW_H
