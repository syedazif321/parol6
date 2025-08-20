#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QLineEdit>
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
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit_msgs/srv/get_position_ik.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

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

    void on_btnShowJointValues_clicked();
    void on_btnShowPoseValues_clicked();
    void on_btnSaveTarget_clicked();      
    void on_btnSaveTarget_2_clicked();     
    void on_btnGoToTarget_clicked();

    void on_btnServoOn_clicked();
    void on_btnServoOff_clicked();

    void onJogButtonPressed();
    void onJogButtonReleased();

    void on_sliderSpeed_valueChanged(int value);

    void on_btnStartProgram_clicked();
    void on_btnStopProgram_clicked();

    void on_btnApplyJointPositions_clicked();
    void on_btnMoveToPose_clicked();

private:
    Ui::MainWindow *ui;

    QString kTargetFilePath;

   
    std::vector<double> current_joint_values;       
    std::vector<double> current_pose_values;       


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
    bool selectTargetAndPreviewInGUI(const QString& name); 

    bool fetchCurrentPoseRPY(PoseRPY &out);
    bool computeIKToJoints(const PoseRPY &target_pose, std::vector<double> &out_joints);


    static void rpyToQuat(double r, double p, double y, double &qx, double &qy, double &qz, double &qw);
    static void quatToRpy(double qx, double qy, double qz, double qw, double &r, double &p, double &y);

    void addTargetNameToCombo(const QString& name);
    bool parseFlatArrayTarget(const QString& key, const QJsonArray& arr);
    void showPreviewText(const QString& title, const std::vector<double>& vals);
    void showPreviewTextPose(const QString& title, const PoseRPY& pose);


    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr jog_publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_on_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_off_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_client_;


    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_pipeline_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_pipeline_client_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string base_frame_ = "base_link";
    std::string ee_link_ = "tool0"; // set to your EE frame
    std::vector<std::string> joint_names_ { "J1","J2","J3","J4","J5","J6" };
};

#endif // MAINWINDOW_H
