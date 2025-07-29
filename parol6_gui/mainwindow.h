
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <vector>
#include <map>
#include <memory>
#include <mutex>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    void on_btnSaveJointTarget_clicked();
    void on_btnSavePoseTarget_clicked();
    void on_btnGoToTarget_clicked();

    void on_btnServoOn_clicked();
    void on_btnServoOff_clicked();

    void onJogButtonPressed();
    void onJogButtonReleased();

private:
    Ui::MainWindow *ui;

    QString kTargetFilePath;

    std::vector<double> current_joint_values;
    std::map<QString, std::vector<double>> saved_joint_targets;
    std::map<QString, QString> saved_pose_targets;

    void updateJointLabels();
    void sendJogCommand(double x, double y, double z, double rx, double ry, double rz);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void loadTargetsFromJson();
    void saveTargetsToJson();

    // ROS 2 Node and communication
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr jog_publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_on_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_off_client_;

};

#endif // MAINWINDOW_H