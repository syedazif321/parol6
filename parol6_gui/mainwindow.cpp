// jog + servo control additions only
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <QPushButton>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      current_joint_values(6, 0.0)
{
    ui->setupUi(this);

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

void MainWindow::on_btnShowJointValues_clicked() {
    // TODO: implement joint display logic
}

void MainWindow::on_btnShowPoseValues_clicked() {
    // TODO: implement pose display logic
}

void MainWindow::on_btnSaveJointTarget_clicked() {
    // TODO: save current joint target
}

void MainWindow::on_btnSavePoseTarget_clicked() {
    // TODO: save current pose target
}

void MainWindow::on_btnGoToTarget_clicked() {
    // TODO: send robot to selected target
}