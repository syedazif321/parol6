#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <vector>
#include <map>
#include <memory>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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

private:
    Ui::MainWindow *ui;

    std::vector<double> current_joint_values;
    std::map<QString, std::vector<double>> saved_joint_targets;
    std::map<QString, QString> saved_pose_targets;

    void updateJointLabels();

    // ROS 2 Node and subscription
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread ros_spin_thread_;
    std::mutex joint_state_mutex_;

    void startROSNode();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void updateFromROS();
};

#endif // MAINWINDOW_H
