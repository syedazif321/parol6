#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      current_joint_values(6, 0.0)
{
    ui->setupUi(this);

    connect(ui->btnShowJointValues, &QPushButton::clicked, this, &MainWindow::on_btnShowJointValues_clicked);
    connect(ui->btnShowPoseValues, &QPushButton::clicked, this, &MainWindow::on_btnShowPoseValues_clicked);
    connect(ui->btnSaveJointTarget, &QPushButton::clicked, this, &MainWindow::on_btnSaveJointTarget_clicked);
    connect(ui->btnSavePoseTarget, &QPushButton::clicked, this, &MainWindow::on_btnSavePoseTarget_clicked);
    connect(ui->btnGoToTarget, &QPushButton::clicked, this, &MainWindow::on_btnGoToTarget_clicked);

    // Initialize ROS 2
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    ros_node_ = rclcpp::Node::make_shared("parol6_gui_node");

    joint_state_sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&MainWindow::jointStateCallback, this, std::placeholders::_1));

    // Timer to periodically call spin_some
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

void MainWindow::on_btnShowJointValues_clicked() {
    updateJointLabels();
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

void MainWindow::on_btnShowPoseValues_clicked() {
    ui->textTargetValues->setText("Position: x=0.5, y=0.2, z=0.1\nOrientation: x=0.0, y=0.0, z=0.0, w=1.0");
}

void MainWindow::on_btnSaveJointTarget_clicked() {
    QString name = ui->lineEditTargetName->text();
    if (!name.isEmpty()) {
        saved_joint_targets[name] = current_joint_values;
        ui->comboBoxTargets->addItem(name);
    }
}

void MainWindow::on_btnSavePoseTarget_clicked() {
    QString name = ui->lineEditTargetName->text();
    if (!name.isEmpty()) {
        QString pose = ui->textTargetValues->toPlainText();
        saved_pose_targets[name] = pose;
        ui->comboBoxTargets->addItem(name);
    }
}

void MainWindow::on_btnGoToTarget_clicked() {
    QString selected = ui->comboBoxTargets->currentText();
    if (saved_joint_targets.contains(selected)) {
        current_joint_values = saved_joint_targets[selected];
        updateJointLabels();
    } else if (saved_pose_targets.contains(selected)) {
        ui->textTargetValues->setText(saved_pose_targets[selected]);
    } else {
        QMessageBox::warning(this, "Target not found", "The selected target does not exist.");
    }
}
