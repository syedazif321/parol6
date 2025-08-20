#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QTimer>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDir>
#include <QDebug>
#include <cmath>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>

// Guard TF2 includes so build never breaks if headers aren't found
#if __has_include(<tf2_ros/transform_listener.h>)
  #define HAVE_TF2 1
  #include <tf2_ros/transform_listener.h>
  #include <tf2_ros/buffer.h>
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
  #include <tf2/LinearMath/Quaternion.h>
  #include <tf2/LinearMath/Matrix3x3.h>
#else
  #define HAVE_TF2 0
#endif

static inline double deg2rad(double d){ return d * M_PI / 180.0; }
static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      current_joint_values(6, 0.0),
      current_pose_values(6, 0.0)
{
    ui->setupUi(this);

    QString basePath = QDir::cleanPath(QCoreApplication::applicationDirPath() + "/../../..");
    kTargetFilePath = basePath + "/parol6/robot_data/Targets.json";

    loadTargetsFromJson();

    // Existing connections
    connect(ui->btnShowJointValues, &QPushButton::clicked, this, &MainWindow::on_btnShowJointValues_clicked);
    connect(ui->btnShowPoseValues,  &QPushButton::clicked, this, &MainWindow::on_btnShowPoseValues_clicked);
    connect(ui->btnSaveTarget,      &QPushButton::clicked, this, &MainWindow::on_btnSaveTarget_clicked);
    connect(ui->btnGoToTarget,      &QPushButton::clicked, this, &MainWindow::on_btnGoToTarget_clicked);
    connect(ui->btnServoOn,         &QPushButton::clicked, this, &MainWindow::on_btnServoOn_clicked);
    connect(ui->btnServoOff,        &QPushButton::clicked, this, &MainWindow::on_btnServoOff_clicked);
    connect(ui->sliderSpeed,        &QSlider::valueChanged, this, &MainWindow::on_sliderSpeed_valueChanged);

    // Program control buttons
    connect(ui->btnStartProgram, &QPushButton::clicked, this, &MainWindow::on_btnStartProgram_clicked);
    connect(ui->btnSopProgram,   &QPushButton::clicked, this, &MainWindow::on_btnSopProgram_clicked);

    // Joint & pose apply buttons
    connect(ui->btnApplyJointPositions, &QPushButton::clicked, this, &MainWindow::on_btnApplyJointPositions_clicked);
    connect(ui->btnMoveToPose,          &QPushButton::clicked, this, &MainWindow::on_btnMoveToPose_clicked);

    // Jog buttons
    auto jogButtons = {
        ui->btnJogXPlus, ui->btnJogXMinus, ui->btnJogYPlus, ui->btnJogYMinus,
        ui->btnJogZPlus, ui->btnJogZMinus, ui->btnJogRPlus, ui->btnJogRMinus,
        ui->btnJogPPlus, ui->btnJogPMinus, ui->btnJogYawPlus, ui->btnJogYawMinus
    };
    for (QPushButton *btn : jogButtons) {
        connect(btn, &QPushButton::pressed, this, &MainWindow::onJogButtonPressed);
        connect(btn, &QPushButton::released, this, &MainWindow::onJogButtonReleased);
    }

    // Initialize ROS 2
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    ros_node_ = rclcpp::Node::make_shared("parol6_gui_node");

    joint_state_sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&MainWindow::jointStateCallback, this, std::placeholders::_1));

    jog_publisher_ = ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);

    servo_on_client_  = ros_node_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_off_client_ = ros_node_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
    trajectory_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        ros_node_, "/arm_controller/follow_joint_trajectory");

    // start/stop pipeline & IK
    start_pipeline_client_ = ros_node_->create_client<std_srvs::srv::Trigger>("/start_pipeline");
    stop_pipeline_client_  = ros_node_->create_client<std_srvs::srv::Trigger>("/stop_pipeline");
    compute_ik_client_     = ros_node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

#if HAVE_TF2
    // TF for EE pose
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(ros_node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
#endif

    // Spin timer
    QTimer *rosTimer = new QTimer(this);
    connect(rosTimer, &QTimer::timeout, this, [this]() { rclcpp::spin_some(ros_node_); });
    rosTimer->start(50);
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

// Program control
void MainWindow::on_btnStartProgram_clicked(){
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (!start_pipeline_client_->wait_for_service(std::chrono::seconds(2))) {
        QMessageBox::warning(this,"Service","/start_pipeline not available");
        return;
    }
    start_pipeline_client_->async_send_request(req);
}

void MainWindow::on_btnSopProgram_clicked(){
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (!stop_pipeline_client_->wait_for_service(std::chrono::seconds(2))) {
        QMessageBox::warning(this,"Service","/stop_pipeline not available");
        return;
    }
    stop_pipeline_client_->async_send_request(req);
}

// Jog
void MainWindow::onJogButtonPressed() {
    QPushButton *btn = qobject_cast<QPushButton *>(sender());
    if (!btn) return;
    double x=0, y=0, z=0, rx=0, ry=0, rz=0;
    QString name = btn->objectName();

    if (name=="btnJogXPlus") x=0.1;
    else if (name=="btnJogXMinus") x=-0.1;
    else if (name=="btnJogYPlus") y=0.1;
    else if (name=="btnJogYMinus") y=-0.1;
    else if (name=="btnJogZPlus") z=0.1;
    else if (name=="btnJogZMinus") z=-0.1;
    else if (name=="btnJogRPlus") rx=0.1;
    else if (name=="btnJogRMinus") rx=-0.1;
    else if (name=="btnJogPPlus") ry=0.1;
    else if (name=="btnJogPMinus") ry=-0.1;
    else if (name=="btnJogYawPlus") rz=0.1;
    else if (name=="btnJogYawMinus") rz=-0.1;

    sendJogCommand(x, y, z, rx, ry, rz);
}

void MainWindow::onJogButtonReleased() {
    sendJogCommand(0,0,0,0,0,0);
}

void MainWindow::sendJogCommand(double x,double y,double z,double rx,double ry,double rz){
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = ros_node_->get_clock()->now();
    msg.header.frame_id = base_frame_;
    msg.twist.linear.x = x; msg.twist.linear.y = y; msg.twist.linear.z = z;
    msg.twist.angular.x = rx; msg.twist.angular.y = ry; msg.twist.angular.z = rz;
    jog_publisher_->publish(msg);
}

// Joint states
void MainWindow::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    if (msg->position.size()>=6){
        for(size_t i=0;i<6;i++) current_joint_values[i]=msg->position[i];
    }
}

// Labels
void MainWindow::updateJointLabels(){
    for(int i=0;i<6;i++){
        if (auto *lbl = findChild<QLabel*>("valueJoint" + QString::number(i+1))) {
            lbl->setText(QString::number(current_joint_values[i], 'f', 3));
        }
    }
}

// NOTE: pose-value labels are valueJoint1_3..valueJoint6_3 for X,Y,Z,R,P,Y
void MainWindow::updatePoseLabels(){
    auto setLbl = [&](const char* name, double val){
        if (auto *lbl = findChild<QLabel*>(name)) lbl->setText(QString::number(val,'f',3));
    };
    setLbl("valueJoint1_3", current_pose_values[0]);                 // X (m)
    setLbl("valueJoint2_3", current_pose_values[1]);                 // Y (m)
    setLbl("valueJoint3_3", current_pose_values[2]);                 // Z (m)
    setLbl("valueJoint4_3", rad2deg(current_pose_values[3]));        // Roll (deg)
    setLbl("valueJoint5_3", rad2deg(current_pose_values[4]));        // Pitch (deg)
    setLbl("valueJoint6_3", rad2deg(current_pose_values[5]));        // Yaw (deg)
}

bool MainWindow::fetchCurrentPoseRPY(PoseRPY &out){
#if HAVE_TF2
    try{
        auto tf = tf_buffer_->lookupTransform(base_frame_, ee_link_, tf2::TimePointZero, tf2::durationFromSec(0.2));
        out.x = tf.transform.translation.x;
        out.y = tf.transform.translation.y;
        out.z = tf.transform.translation.z;

        tf2::Quaternion q;
        tf2::fromMsg(tf.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        double r,p,y;
        m.getRPY(r,p,y);
        out.roll = r; out.pitch = p; out.yaw = y;
        return true;
    } catch(const std::exception &ex){
        RCLCPP_WARN(ros_node_->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }
#else
    (void)out;
    RCLCPP_WARN_ONCE(ros_node_->get_logger(), "TF2 headers not found at build time; pose will use spin boxes / saved values.");
    return false;
#endif
}

// ---------- Targets (load/save) ----------
void MainWindow::loadTargetsFromJson(){
    saved_joint_targets.clear();
    saved_pose_targets.clear();

    QFile file(kTargetFilePath);
    if(!file.open(QIODevice::ReadOnly)) return;
    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    QJsonObject root = doc.object();

    // Backward-compat: flat format { "Target":[J1..J6], ... }
    for(const QString &k : root.keys()){
        if (root[k].isArray()){
            QJsonArray arr = root[k].toArray();
            std::vector<double> joints;
            joints.reserve(arr.size());
            for(const auto &v: arr) joints.push_back(v.toDouble());
            saved_joint_targets[k] = joints;
            if (ui->comboBoxTargets->findText(k) < 0) ui->comboBoxTargets->addItem(k);
        }
    }
    // New structured
    if (root.contains("joints") && root["joints"].isObject()){
        auto jmap = root["joints"].toObject();
        for (auto it = jmap.begin(); it != jmap.end(); ++it){
            QJsonArray arr = it.value().toArray();
            std::vector<double> joints;
            joints.reserve(arr.size());
            for(const auto &v: arr) joints.push_back(v.toDouble());
            saved_joint_targets[it.key()] = joints;
            if (ui->comboBoxTargets->findText(it.key()) < 0)
                ui->comboBoxTargets->addItem(it.key());
        }
    }
    if (root.contains("poses") && root["poses"].isObject()){
        auto pmap = root["poses"].toObject();
        for (auto it = pmap.begin(); it != pmap.end(); ++it){
            auto pobj = it.value().toObject();
            auto posA = pobj["position"].toArray();
            auto ornA = pobj["orientation"].toArray(); // roll,pitch,yaw (deg)
            PoseRPY p{};
            p.x = posA.size()>0 ? posA[0].toDouble() : 0.0;
            p.y = posA.size()>1 ? posA[1].toDouble() : 0.0;
            p.z = posA.size()>2 ? posA[2].toDouble() : 0.0;
            p.roll  = deg2rad(ornA.size()>0 ? ornA[0].toDouble() : 0.0);
            p.pitch = deg2rad(ornA.size()>1 ? ornA[1].toDouble() : 0.0);
            p.yaw   = deg2rad(ornA.size()>2 ? ornA[2].toDouble() : 0.0);
            saved_pose_targets[it.key()] = p;
            if (ui->comboBoxTargets->findText(it.key()) < 0)
                ui->comboBoxTargets->addItem(it.key());
        }
    }
}

void MainWindow::saveTargetsToJson(){
    QJsonObject root;

    // joints
    QJsonObject jointsObj;
    for(const auto &pair: saved_joint_targets){
        QJsonArray arr;
        for(double val: pair.second) arr.append(val);
        jointsObj.insert(pair.first, arr);
    }
    root.insert("joints", jointsObj);

    // poses (store orientation in degrees)
    QJsonObject posesObj;
    for (const auto &pair : saved_pose_targets){
        const PoseRPY &p = pair.second;
        QJsonArray posA; posA.append(p.x); posA.append(p.y); posA.append(p.z);
        QJsonArray ornA; ornA.append(rad2deg(p.roll)); ornA.append(rad2deg(p.pitch)); ornA.append(rad2deg(p.yaw));
        QJsonObject pobj; pobj.insert("position", posA); pobj.insert("orientation", ornA);
        posesObj.insert(pair.first, pobj);
    }
    root.insert("poses", posesObj);

    QJsonDocument doc(root);
    QFile file(kTargetFilePath);
    QDir().mkpath(QFileInfo(file).absolutePath());
    if(file.open(QIODevice::WriteOnly)){
        file.write(doc.toJson(QJsonDocument::Indented));
        file.close();
    }
}

// Save Target (Joint + Pose)
void MainWindow::on_btnSaveTarget_clicked(){
    QString name = ui->lineEditTargetName->text();
    if(name.isEmpty()) return;

    // Save joints
    saved_joint_targets[name] = current_joint_values;

    // Save pose (prefer TF; fallback to spin boxes)
    PoseRPY p{};
    if (!fetchCurrentPoseRPY(p)){
        p.x = ui->spinX->value();
        p.y = ui->spinY->value();
        p.z = ui->spinZ->value();
        p.roll  = deg2rad(ui->spinRoll->value());
        p.pitch = deg2rad(ui->spinPitch->value());
        p.yaw   = deg2rad(ui->spinYaw->value());
    }
    saved_pose_targets[name] = p;

    if (ui->comboBoxTargets->findText(name) < 0)
        ui->comboBoxTargets->addItem(name);

    ui->lineEditTargetName->clear();
    saveTargetsToJson();
}

void MainWindow::on_btnGoToTarget_clicked(){
    QString name = ui->comboBoxTargets->currentText();

    // C++17: use find(), not contains()
    if (saved_joint_targets.find(name) != saved_joint_targets.end()){
        sendTrajectoryToTarget(saved_joint_targets[name]);
        return;
    }
    if (saved_pose_targets.find(name) != saved_pose_targets.end()){
        std::vector<double> joints;
        if (computeIKToJoints(saved_pose_targets[name], joints)){
            sendTrajectoryToTarget(joints);
            return;
        }
    }
    QMessageBox::warning(this,"Invalid Target","Selected target not found.");
}

void MainWindow::on_btnShowJointValues_clicked(){
    updateJointLabels();
}

void MainWindow::on_btnShowPoseValues_clicked(){
    PoseRPY p{};
    if (fetchCurrentPoseRPY(p)){
        current_pose_values = { p.x, p.y, p.z, p.roll, p.pitch, p.yaw };
    } else {
        current_pose_values = {
            ui->spinX->value(),
            ui->spinY->value(),
            ui->spinZ->value(),
            deg2rad(ui->spinRoll->value()),
            deg2rad(ui->spinPitch->value()),
            deg2rad(ui->spinYaw->value())
        };
    }
    updatePoseLabels();
}

// Speed slider
void MainWindow::on_sliderSpeed_valueChanged(int value){
    double speed = value/100.0;
    ui->labelCurrentSpeed->setText(QString::number(speed,'f',2)+"x");
}

// ---------- Send trajectory ----------
void MainWindow::sendTrajectoryToTarget(const std::vector<double>& joint_positions){
    if(!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(2))){
        RCLCPP_ERROR(ros_node_->get_logger(),"Trajectory action server not available.");
        QMessageBox::warning(this,"Controller","/arm_controller/follow_joint_trajectory not available");
        return;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;

    double speed = ui->sliderSpeed->value() / 100.0; // 0.5x .. 2.0x
    double t = std::max(1.0, 2.0 / speed);
    point.time_from_start = rclcpp::Duration::from_seconds(t);

    goal.trajectory.points.clear();
    goal.trajectory.points.push_back(point);

    auto goal_handle_future = trajectory_action_client_->async_send_goal(goal);
    if(rclcpp::spin_until_future_complete(ros_node_, goal_handle_future)==rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(ros_node_->get_logger(),"Trajectory goal sent successfully.");
    } else {
        RCLCPP_ERROR(ros_node_->get_logger(),"Failed to send trajectory goal.");
    }
}

// Apply joints from spin boxes
void MainWindow::on_btnApplyJointPositions_clicked(){
    std::vector<double> target(6,0.0);
    target[0] = ui->spinJoint1->value();
    target[1] = ui->spinJoint2->value();
    target[2] = ui->spinJoint3->value();
    target[3] = ui->spinJoint4->value();
    target[4] = ui->spinJoint5->value();
    target[5] = ui->spinJoint6->value();
    sendTrajectoryToTarget(target);
}

// Move EE using IK from X/Y/Z/R/P/Y spin boxes (RPY in degrees)
void MainWindow::on_btnMoveToPose_clicked(){
    PoseRPY p{};
    p.x = ui->spinX->value();
    p.y = ui->spinY->value();
    p.z = ui->spinZ->value();
    p.roll  = deg2rad(ui->spinRoll->value());
    p.pitch = deg2rad(ui->spinPitch->value());
    p.yaw   = deg2rad(ui->spinYaw->value());

    std::vector<double> joints;
    if (!computeIKToJoints(p, joints)){
        QMessageBox::warning(this, "IK", "IK failed for requested pose.");
        return;
    }
    sendTrajectoryToTarget(joints);
}

// ---------- IK helper ----------
bool MainWindow::computeIKToJoints(const PoseRPY &target_pose, std::vector<double> &out_joints){
    if (!compute_ik_client_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_ERROR(ros_node_->get_logger(),"compute_ik service not available.");
        return false;
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = ros_node_->now();
    ps.header.frame_id = base_frame_;
    ps.pose.position.x = target_pose.x;
    ps.pose.position.y = target_pose.y;
    ps.pose.position.z = target_pose.z;

#if HAVE_TF2
    tf2::Quaternion q;
    q.setRPY(target_pose.roll, target_pose.pitch, target_pose.yaw);
    ps.pose.orientation = tf2::toMsg(q);
#else
    // minimal orientation fallback if TF2 headers absent
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = 0.0;
    ps.pose.orientation.w = 1.0;
#endif

    moveit_msgs::msg::RobotState seed;
    seed.joint_state.name = joint_names_;
    seed.joint_state.position.resize(6);
    for (size_t i=0;i<6;i++) seed.joint_state.position[i] = current_joint_values[i];

    auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    req->ik_request.group_name = "arm"; // change if your MoveIt group differs
    req->ik_request.robot_state = seed;
    req->ik_request.ik_link_name = ee_link_;
    req->ik_request.pose_stamped = ps;
    req->ik_request.timeout = rclcpp::Duration::from_seconds(0.5);
    // no attempts field on Humble

    auto future = compute_ik_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(ros_node_, future) != rclcpp::FutureReturnCode::SUCCESS){
        return false;
    }
    auto resp = future.get();
    if (resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
        RCLCPP_WARN(ros_node_->get_logger(),"IK error code: %d", resp->error_code.val);
        return false;
    }

    const auto &js = resp->solution.joint_state;
    std::vector<double> result(6,0.0);
    for (size_t i=0;i<joint_names_.size(); ++i){
        auto it = std::find(js.name.begin(), js.name.end(), joint_names_[i]);
        if (it != js.name.end()){
            size_t idx = std::distance(js.name.begin(), it);
            if (idx < js.position.size()) result[i] = js.position[idx];
        } else {
            result[i] = current_joint_values[i];
        }
    }
    out_joints = std::move(result);
    return true;
}
