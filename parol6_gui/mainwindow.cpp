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
#include <algorithm>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>




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

namespace {
static QString normalizedName(const QString& raw) {
    QString s = raw;
    s.replace(QChar(0x00A0), QChar(' ')); 
    s.replace(QChar(0x2007), QChar(' ')); 
    s.replace(QChar(0x202F), QChar(' ')); 
    return s.trimmed();
}
} 
void MainWindow::rpyToQuat(double r, double p, double y, double &qx, double &qy, double &qz, double &qw){

    double cy = std::cos(y * 0.5);
    double sy = std::sin(y * 0.5);
    double cp = std::cos(p * 0.5);
    double sp = std::sin(p * 0.5);
    double cr = std::cos(r * 0.5);
    double sr = std::sin(r * 0.5);
    qw = cr*cp*cy + sr*sp*sy;
    qx = sr*cp*cy - cr*sp*sy;
    qy = cr*sp*cy + sr*cp*sy;
    qz = cr*cp*sy - sr*sp*cy;
}

void MainWindow::quatToRpy(double qx, double qy, double qz, double qw, double &r, double &p, double &y){

    double sinr_cosp = 2.0 * (qw*qx + qy*qz);
    double cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy);
    r = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qw*qy - qz*qx);
    if (std::abs(sinp) >= 1.0)
        p = std::copysign(M_PI/2.0, sinp);
    else
        p = std::asin(sinp);
    double siny_cosp = 2.0 * (qw*qz + qx*qy);
    double cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);
    y = std::atan2(siny_cosp, cosy_cosp);
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      current_joint_values(6, 0.0),
      current_pose_values(6, 0.0)
{
    ui->setupUi(this);

    ui->spinRoll->setRange(-M_PI, M_PI);
    ui->spinPitch->setRange(-M_PI, M_PI);
    ui->spinYaw->setRange(-M_PI, M_PI);
    ui->spinRoll->setDecimals(3);
    ui->spinPitch->setDecimals(3);
    ui->spinYaw->setDecimals(3);

    ui->spinX->setRange(-10.0, 10.0);
    ui->spinY->setRange(-10.0, 10.0);
    ui->spinZ->setRange(-10.0, 10.0);
    ui->spinX->setDecimals(3);
    ui->spinY->setDecimals(3);
    ui->spinZ->setDecimals(3);

    QString basePath = QDir::cleanPath(QCoreApplication::applicationDirPath() + "/../../..");
    kTargetFilePath = basePath + "/parol6/robot_data/Targets.json";

    {
        auto jogButtons = {
            ui->btnJogXPlus, ui->btnJogXMinus, ui->btnJogYPlus, ui->btnJogYMinus,
            ui->btnJogZPlus, ui->btnJogZMinus, ui->btnJogRPlus, ui->btnJogRMinus,
            ui->btnJogPPlus, ui->btnJogPMinus, ui->btnJogYawPlus, ui->btnJogYawMinus
        };
        for (QPushButton *btn : jogButtons) {
            connect(btn, &QPushButton::pressed, this, &MainWindow::onJogButtonPressed, Qt::UniqueConnection);
            connect(btn, &QPushButton::released, this, &MainWindow::onJogButtonReleased, Qt::UniqueConnection);
        }
    }

    connect(ui->comboBoxTargets, &QComboBox::currentTextChanged, this,
            [this](const QString& t){ ui->lineEditTargetName->setText(t); });

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

    speed_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>("/pipeline/speed", 10);


    // Subscriber for box count
    box_count_sub_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
        "/pipeline/box_count", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            QString text = QString("Boxes: %1").arg(msg->data);
            // update GUI from ROS thread safely
            QMetaObject::invokeMethod(this, [this, text]() {
                ui->labelBoxCount->setText(text);
            });
        });


    // start/stop pipeline & IK
    start_pipeline_client_ = ros_node_->create_client<std_srvs::srv::Trigger>("/start_pipeline");
    stop_pipeline_client_  = ros_node_->create_client<std_srvs::srv::Trigger>("/stop_pipeline");
    compute_ik_client_     = ros_node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

#if HAVE_TF2
    // TF for EE pose
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(ros_node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
#endif

    QTimer *rosTimer = new QTimer(this);
    connect(rosTimer, &QTimer::timeout, this, [this]() { rclcpp::spin_some(ros_node_); });
    rosTimer->start(50);

    loadTargetsFromJson();

    on_sliderSpeed_valueChanged(ui->sliderSpeed->value());
}

MainWindow::~MainWindow() {
    rclcpp::shutdown();
    delete ui;
}

void MainWindow::on_btnServoOn_clicked() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (!servo_on_client_->wait_for_service(std::chrono::seconds(1))) {
        QMessageBox::warning(this,"Service","/start_servo not available");
        return;
    }
    servo_on_client_->async_send_request(request);
}

void MainWindow::on_btnServoOff_clicked() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (!servo_off_client_->wait_for_service(std::chrono::seconds(1))) {
        QMessageBox::warning(this,"Service","/stop_servo not available");
        return;
    }
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

void MainWindow::on_btnStopProgram_clicked(){
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (!stop_pipeline_client_->wait_for_service(std::chrono::seconds(2))) {
        QMessageBox::warning(this,"Service","/stop_pipeline not available");
        return;
    }
    stop_pipeline_client_->async_send_request(req);
}

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

void MainWindow::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    if (msg->position.size()>=6){
        for(size_t i=0;i<6;i++) current_joint_values[i]=msg->position[i];
    }
}

void MainWindow::updateJointLabels(){
    for(int i=0;i<6;i++){
        if (auto *lbl = findChild<QLabel*>("valueJoint" + QString::number(i+1))) {
            lbl->setText(QString::number(current_joint_values[i], 'f', 3));
        }
    }
}

void MainWindow::updatePoseLabels(){
    auto setLbl = [&](const char* name, double val){
        if (auto *lbl = findChild<QLabel*>(name)) lbl->setText(QString::number(val,'f',3));
    };
    setLbl("valueJoint1_3", current_pose_values[0]); // X (m)
    setLbl("valueJoint2_3", current_pose_values[1]); // Y (m)
    setLbl("valueJoint3_3", current_pose_values[2]); // Z (m)
    setLbl("valueJoint4_3", current_pose_values[3]); // Roll RAD
    setLbl("valueJoint5_3", current_pose_values[4]); // Pitch RAD
    setLbl("valueJoint6_3", current_pose_values[5]); // Yaw RAD
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
        m.getRPY(out.roll, out.pitch, out.yaw);
        return true;
    } catch(const std::exception &ex){
        RCLCPP_WARN(ros_node_->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }
#else
    (void)out;
    RCLCPP_WARN_ONCE(ros_node_->get_logger(), "TF2 headers not found; pose uses spin boxes/saved values.");
    return false;
#endif
}


void MainWindow::on_btnShowJointValues_clicked(){
    updateJointLabels();
}

void MainWindow::on_btnShowPoseValues_clicked(){
    PoseRPY p{};
    if (fetchCurrentPoseRPY(p)){
        current_pose_values = { p.x, p.y, p.z, p.roll, p.pitch, p.yaw }; // radians
    } else {
        current_pose_values = {
            ui->spinX->value(),
            ui->spinY->value(),
            ui->spinZ->value(),
            ui->spinRoll->value(),
            ui->spinPitch->value(),
            ui->spinYaw->value()
        };
    }
    updatePoseLabels();
}

void MainWindow::on_sliderSpeed_valueChanged(int value){
    double speed = value / 100.0;

    ui->labelCurrentSpeed->setText(QString::number(speed,'f',2)+"x");
    ui->labelActiveSpeed->setText(QString("Speed: %1x").arg(QString::number(speed,'f',2)));
    auto msg = std_msgs::msg::Float64();
    msg.data = speed;
    speed_pub_->publish(msg);
}



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

void MainWindow::on_btnMoveToPose_clicked(){
    PoseRPY p{};
    p.x = ui->spinX->value();
    p.y = ui->spinY->value();
    p.z = ui->spinZ->value();
    p.roll  = ui->spinRoll->value();
    p.pitch = ui->spinPitch->value();
    p.yaw   = ui->spinYaw->value();

    std::vector<double> joints;
    if (!computeIKToJoints(p, joints)){
        QMessageBox::warning(this, "IK", "IK failed for requested pose.");
        return;
    }
    sendTrajectoryToTarget(joints);
}

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
    double qx,qy,qz,qw;
    rpyToQuat(target_pose.roll, target_pose.pitch, target_pose.yaw, qx,qy,qz,qw);
    ps.pose.orientation.x = qx;
    ps.pose.orientation.y = qy;
    ps.pose.orientation.z = qz;
    ps.pose.orientation.w = qw;
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


void MainWindow::addTargetNameToCombo(const QString& name){
    if (ui->comboBoxTargets->findText(name) < 0)
        ui->comboBoxTargets->addItem(name);
}

bool MainWindow::parseFlatArrayTarget(const QString& key, const QJsonArray& arr){

    if (arr.size()==6){
        std::vector<double> j(6,0.0);
        for(int i=0;i<6;i++) j[i] = arr[i].toDouble();
        saved_joint_targets[key] = j;
        addTargetNameToCombo(key);
        return true;
    } else if (arr.size()==7){
        PoseRPY p{};
        double qx = arr[3].toDouble();
        double qy = arr[4].toDouble();
        double qz = arr[5].toDouble();
        double qw = arr[6].toDouble();
        p.x = arr[0].toDouble();
        p.y = arr[1].toDouble();
        p.z = arr[2].toDouble();
        quatToRpy(qx,qy,qz,qw, p.roll,p.pitch,p.yaw);
        saved_pose_targets[key] = p;
        addTargetNameToCombo(key);
        return true;
    }
    return false;
}

void MainWindow::loadTargetsFromJson(){
    saved_joint_targets.clear();
    saved_pose_targets.clear();

    QFile file(kTargetFilePath);
    if(!file.open(QIODevice::ReadOnly)) return;
    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    QJsonObject root = doc.object();

    for(const QString &k : root.keys()){
        if (k=="joints" || k=="poses") continue;
        if (root[k].isArray()){
            if (parseFlatArrayTarget(k, root[k].toArray())) continue;
        }
    }

    if (root.contains("joints") && root["joints"].isObject()){
        auto jmap = root["joints"].toObject();
        for (auto it = jmap.begin(); it != jmap.end(); ++it){
            QJsonArray arr = it.value().toArray();
            if (arr.size()==6){
                std::vector<double> joints(6,0.0);
                for(int i=0;i<6;i++) joints[i]=arr[i].toDouble();
                saved_joint_targets[it.key()] = joints;
                addTargetNameToCombo(it.key());
            }
        }
    }

    if (root.contains("poses") && root["poses"].isObject()){
        auto pmap = root["poses"].toObject();
        for (auto it = pmap.begin(); it != pmap.end(); ++it){
            auto pobj = it.value().toObject();
            PoseRPY p{};
            if (pobj.contains("position") && pobj["position"].isArray()){
                auto posA = pobj["position"].toArray();
                p.x = posA.size()>0 ? posA[0].toDouble() : 0.0;
                p.y = posA.size()>1 ? posA[1].toDouble() : 0.0;
                p.z = posA.size()>2 ? posA[2].toDouble() : 0.0;
            }
            if (pobj.contains("orientation") && pobj["orientation"].isArray()){
                auto ornA = pobj["orientation"].toArray();
                if (ornA.size()==4){
                    double qx=ornA[0].toDouble(), qy=ornA[1].toDouble(), qz=ornA[2].toDouble(), qw=ornA[3].toDouble();
                    quatToRpy(qx, qy, qz, qw, p.roll, p.pitch, p.yaw);
                } else if (ornA.size()==3){

                    p.roll  = ornA[0].toDouble();
                    p.pitch = ornA[1].toDouble();
                    p.yaw   = ornA[2].toDouble();
                }
            }
            saved_pose_targets[it.key()] = p;
            addTargetNameToCombo(it.key());
        }
    }
}

void MainWindow::saveTargetsToJson(){
    QJsonObject root;
    {
        QFile f(kTargetFilePath);
        if (f.open(QIODevice::ReadOnly)) {
            root = QJsonDocument::fromJson(f.readAll()).object();
            f.close();
        }
    }

    QJsonObject jointsObj;
    for(const auto &pair: saved_joint_targets){
        QJsonArray arr;
        for(double val: pair.second) arr.append(val);
        jointsObj.insert(pair.first, arr);
    }
    root.insert("joints", jointsObj);

    QJsonObject posesObj;
    for (const auto &pair : saved_pose_targets){
        const PoseRPY &p = pair.second;
        double qx,qy,qz,qw;
        rpyToQuat(p.roll, p.pitch, p.yaw, qx,qy,qz,qw);

        QJsonArray posA; posA.append(p.x); posA.append(p.y); posA.append(p.z);
        QJsonArray ornA; ornA.append(qx); ornA.append(qy); ornA.append(qz); ornA.append(qw);
        QJsonObject pobj; pobj.insert("position", posA); pobj.insert("orientation", ornA);
        posesObj.insert(pair.first, pobj);
    }
    root.insert("poses", posesObj);

    QJsonDocument out(root);
    QFile file(kTargetFilePath);
    QDir().mkpath(QFileInfo(file).absolutePath());
    if(file.open(QIODevice::WriteOnly)){
        file.write(out.toJson(QJsonDocument::Indented));
        file.close();
    }
}


void MainWindow::on_btnSaveTarget_clicked(){

    QString name = normalizedName(ui->lineEditTargetName->text());
    if (name.isEmpty()) name = normalizedName(ui->comboBoxTargets->currentText());

    if(name.isEmpty()){
        QMessageBox::warning(this,"Name","Enter a target name or pick one from the dropdown.");
        return;
    }

    std::vector<double> joints(6,0.0);
    if (current_joint_values.size() >= 6){
        joints = current_joint_values;  // radians
    } else {
        joints[0] = ui->spinJoint1->value();
        joints[1] = ui->spinJoint2->value();
        joints[2] = ui->spinJoint3->value();
        joints[3] = ui->spinJoint4->value();
        joints[4] = ui->spinJoint5->value();
        joints[5] = ui->spinJoint6->value();
    }

    saved_joint_targets[name] = joints;
    addTargetNameToCombo(name);

    saveTargetsToJson();
    showPreviewText(QString("Saved Joint Target: %1").arg(name), joints);


    ui->lineEditTargetName->clear();
}


void MainWindow::on_btnSaveTarget_2_clicked(){

    QString name = normalizedName(ui->lineEditTargetName->text());
    if (name.isEmpty()) name = normalizedName(ui->comboBoxTargets->currentText());

    if(name.isEmpty()){
        QMessageBox::warning(this,"Name","Enter a target name or pick one from the dropdown.");
        return;
    }

    PoseRPY p{};
    if (!fetchCurrentPoseRPY(p)){
        p.x = ui->spinX->value();
        p.y = ui->spinY->value();
        p.z = ui->spinZ->value();
        p.roll  = ui->spinRoll->value();
        p.pitch = ui->spinPitch->value();
        p.yaw   = ui->spinYaw->value();
    }

    saved_pose_targets[name] = p;
    addTargetNameToCombo(name);

    // Persist and preview
    saveTargetsToJson();
    showPreviewTextPose(QString("Saved Pose Target: %1").arg(name), p);

    ui->lineEditTargetName->clear();
}


bool MainWindow::selectTargetAndPreviewInGUI(const QString& name){
    auto itJ = saved_joint_targets.find(name);
    if (itJ != saved_joint_targets.end()){
        const auto& j = itJ->second;
        if (j.size()==6){

            ui->spinJoint1->setValue(j[0]);
            ui->spinJoint2->setValue(j[1]);
            ui->spinJoint3->setValue(j[2]);
            ui->spinJoint4->setValue(j[3]);
            ui->spinJoint5->setValue(j[4]);
            ui->spinJoint6->setValue(j[5]);

            showPreviewText(QString("Selected Joint Target: %1").arg(name), j);
            return true;
        }
    }
    auto itP = saved_pose_targets.find(name);
    if (itP != saved_pose_targets.end()){
        const auto& p = itP->second;

        ui->spinX->setValue(p.x);
        ui->spinY->setValue(p.y);
        ui->spinZ->setValue(p.z);
        ui->spinRoll->setValue(p.roll);
        ui->spinPitch->setValue(p.pitch);
        ui->spinYaw->setValue(p.yaw);

        showPreviewTextPose(QString("Selected Pose Target: %1").arg(name), p);
        return true;
    }
    return false;
}

void MainWindow::on_btnGoToTarget_clicked(){
    QString name = ui->comboBoxTargets->currentText();
    if (name.isEmpty()){
        QMessageBox::warning(this,"Target","Select a target.");
        return;
    }

    if (!selectTargetAndPreviewInGUI(name)){
        QMessageBox::warning(this,"Invalid Target","Selected target not found.");
        return;
    }

    if (saved_joint_targets.find(name) != saved_joint_targets.end()){
        sendTrajectoryToTarget(saved_joint_targets[name]);
        return;
    }
    if (saved_pose_targets.find(name) != saved_pose_targets.end()){
        std::vector<double> joints;
        if (computeIKToJoints(saved_pose_targets[name], joints)){
            sendTrajectoryToTarget(joints);
            return;
        } else {
            QMessageBox::warning(this,"IK","IK failed for the pose target.");
            return;
        }
    }
    QMessageBox::warning(this,"Invalid Target","Selected target not found.");
}

void MainWindow::showPreviewText(const QString& title, const std::vector<double>& vals){
    QString s = title + "\n[";
    for(size_t i=0;i<vals.size();++i){
        s += QString::number(vals[i],'f',3);
        if (i+1<vals.size()) s += ", ";
    }
    s += "]";
    ui->textTargetValues->setPlainText(s);
}

void MainWindow::showPreviewTextPose(const QString& title, const PoseRPY& p){
    double qx,qy,qz,qw;
    rpyToQuat(p.roll,p.pitch,p.yaw, qx,qy,qz,qw);
    QString s = title +
        QString("\nX: %1  Y: %2  Z: %3\nRoll: %4  Pitch: %5  Yaw: %6 (rad)\nQuat: [%7, %8, %9, %10]")
            .arg(QString::number(p.x,'f',3))
            .arg(QString::number(p.y,'f',3))
            .arg(QString::number(p.z,'f',3))
            .arg(QString::number(p.roll,'f',3))
            .arg(QString::number(p.pitch,'f',3))
            .arg(QString::number(p.yaw,'f',3))
            .arg(QString::number(qx,'f',6))
            .arg(QString::number(qy,'f',6))
            .arg(QString::number(qz,'f',6))
            .arg(QString::number(qw,'f',6));
    ui->textTargetValues->setPlainText(s);

    current_pose_values = { p.x,p.y,p.z,p.roll,p.pitch,p.yaw };
    updatePoseLabels();
}
