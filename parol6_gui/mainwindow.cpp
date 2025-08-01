// jog + servo control additions only
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <QPushButton>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDir>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>





MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      current_joint_values(6, 0.0)
{
    ui->setupUi(this);

    QString basePath = QDir::cleanPath(QCoreApplication::applicationDirPath() + "/../../..");
    kTargetFilePath = basePath + "/parol6/robot_data/Targets.json";

    qDebug() << "Resolved target path:" << kTargetFilePath;

    loadTargetsFromJson();

    connect(ui->btnShowJointValues, &QPushButton::clicked, this, &MainWindow::updateJointLabels);
    connect(ui->btnShowPoseValues, &QPushButton::clicked, this, &MainWindow::on_btnShowPoseValues_clicked);
    connect(ui->btnSaveJointTarget, &QPushButton::clicked, this, &MainWindow::on_btnSaveJointTarget_clicked);
    connect(ui->btnSavePoseTarget, &QPushButton::clicked, this, &MainWindow::on_btnSavePoseTarget_clicked);
    connect(ui->btnGoToTarget, &QPushButton::clicked, this, &MainWindow::on_btnGoToTarget_clicked);
    connect(ui->btnServoOn, &QPushButton::clicked, this, &MainWindow::on_btnServoOn_clicked);
    connect(ui->btnServoOff, &QPushButton::clicked, this, &MainWindow::on_btnServoOff_clicked);

    // Jog buttons connect to pressed/released signals

    auto jogButtons = {
        ui->btnJogXPlus, ui->btnJogXMinus, ui->btnJogYPlus, ui->btnJogYMinus,
        ui->btnJogZPlus, ui->btnJogZMinus, ui->btnJogRPlus, ui->btnJogRMinus,
        ui->btnJogPPlus, ui->btnJogPMinus, ui->btnJogYPlus1, ui->btnJogYMinus1
    };

    for (QPushButton *btn : jogButtons) {
        connect(btn, &QPushButton::pressed, this, &MainWindow::onJogButtonPressed);
        connect(btn, &QPushButton::released, this, &MainWindow::onJogButtonReleased);
    }

    // Initialize ROS 2
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    ros_node_ = rclcpp::Node::make_shared("parol6_gui_node");

    joint_state_sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&MainWindow::jointStateCallback, this, std::placeholders::_1));

    jog_publisher_ = ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);

    servo_on_client_ = ros_node_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_off_client_ = ros_node_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    trajectory_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        ros_node_, "/arm_controller/follow_joint_trajectory");


    QTimer *rosTimer = new QTimer(this);
    connect(rosTimer, &QTimer::timeout, this, [this]() {
        rclcpp::spin_some(ros_node_);
    });
    rosTimer->start(100);  // 10 Hz
}

MainWindow::~MainWindow() {
    rclcpp::shutdown();
    delete ui;
}

void MainWindow::on_btnServoOn_clicked() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (!servo_on_client_->wait_for_service(std::chrono::seconds(1))) return;
    servo_on_client_->async_send_request(request);
}

void MainWindow::on_btnServoOff_clicked() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (!servo_off_client_->wait_for_service(std::chrono::seconds(1))) return;
    servo_off_client_->async_send_request(request);
}

void MainWindow::onJogButtonPressed() {
    QPushButton *btn = qobject_cast<QPushButton *>(sender());
    if (!btn) return;

    double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
    QString name = btn->objectName();

    if (name == "btnJogXPlus") x = 0.1;
    else if (name == "btnJogXMinus") x = -0.1;
    else if (name == "btnJogYPlus") y = 0.1;
    else if (name == "btnJogYMinus") y = -0.1;
    else if (name == "btnJogZPlus") z = 0.1;
    else if (name == "btnJogZMinus") z = -0.1;
    else if (name == "btnJogRPlus") rx = 0.1;
    else if (name == "btnJogRMinus") rx = -0.1;
    else if (name == "btnJogPPlus") ry = 0.1;
    else if (name == "btnJogPMinus") ry = -0.1;
    else if (name == "btnJogYPlus1") rz = 0.1;
    else if (name == "btnJogYMinus1") rz = -0.1;

    sendJogCommand(x, y, z, rx, ry, rz);
}

void MainWindow::onJogButtonReleased() {
    sendJogCommand(0, 0, 0, 0, 0, 0); // stop
}

void MainWindow::sendJogCommand(double x, double y, double z, double rx, double ry, double rz) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = ros_node_->get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = x;
    msg.twist.linear.y = y;
    msg.twist.linear.z = z;
    msg.twist.angular.x = rx;
    msg.twist.angular.y = ry;
    msg.twist.angular.z = rz;
    jog_publisher_->publish(msg);
}

void MainWindow::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            current_joint_values[i] = msg->position[i];
        }
    }
}

void MainWindow::updateJointLabels() {
    ui->valueJoint1->setText(QString::number(current_joint_values[0], 'f', 2));
    ui->valueJoint2->setText(QString::number(current_joint_values[1], 'f', 2));
    ui->valueJoint3->setText(QString::number(current_joint_values[2], 'f', 2));
    ui->valueJoint4->setText(QString::number(current_joint_values[3], 'f', 2));
    ui->valueJoint5->setText(QString::number(current_joint_values[4], 'f', 2));
    ui->valueJoint6->setText(QString::number(current_joint_values[5], 'f', 2));
}

void MainWindow::loadTargetsFromJson() {
    QFile file(kTargetFilePath);
    if (!file.open(QIODevice::ReadOnly)) return;

    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    QJsonObject obj = doc.object();

    for (const QString& name : obj.keys()) {
        QJsonArray arr = obj[name].toArray();
        std::vector<double> joints;
        for (const auto& val : arr) {
            joints.push_back(val.toDouble());
        }
        saved_joint_targets[name] = joints;
        ui->comboBoxTargets->addItem(name);
    }
}

void MainWindow::saveTargetsToJson() {
    QJsonObject obj;
    for (const auto& pair : saved_joint_targets) {
        QJsonArray arr;
        for (double val : pair.second) {
            arr.append(val);
        }
        obj.insert(pair.first, arr);
    }

    QJsonDocument doc(obj);
    QFile file(kTargetFilePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qDebug() << "Failed to open file for writing:" << kTargetFilePath;
        return;
    }

    file.write(doc.toJson(QJsonDocument::Indented));
    file.close();
    qDebug() << "Successfully wrote to file";

    qDebug() << "Saving to file:" << kTargetFilePath;

}

void MainWindow::sendTrajectoryToTarget(const std::vector<double>& joint_positions)
{
    if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Trajectory action server not available.");
        return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {
        "J1", "J2", "J3", "J4", "J5", "J6"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);

    goal.trajectory.points.push_back(point);

    auto goal_handle_future = trajectory_action_client_->async_send_goal(goal);

    if (rclcpp::spin_until_future_complete(ros_node_, goal_handle_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(ros_node_->get_logger(), "Trajectory goal sent successfully.");
    }
    else
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to send trajectory goal.");
    }
}



void MainWindow::on_btnShowJointValues_clicked() {
    // TODO: implement joint display logic
}

void MainWindow::on_btnShowPoseValues_clicked() {
    // TODO: implement pose display logic
}

void MainWindow::on_btnSaveJointTarget_clicked() {
    QString name = ui->lineEditTargetName->text();
    if (!name.isEmpty()) {
        saved_joint_targets[name] = current_joint_values;
        ui->comboBoxTargets->addItem(name);
        ui->lineEditTargetName->clear();
        saveTargetsToJson();
    }
}


void MainWindow::on_btnSavePoseTarget_clicked() {
    // TODO: save current pose target
}

void MainWindow::on_btnGoToTarget_clicked() {
    QString name = ui->comboBoxTargets->currentText();
    if (saved_joint_targets.contains(name)) {
        sendTrajectoryToTarget(saved_joint_targets[name]);
    } else {
        QMessageBox::warning(this, "Invalid Target", "Selected target not found.");
    }
}

