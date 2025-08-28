
# Parol6 Robotic Arm (ROS 2 Humble)

The **Parol6** project is a complete ROS 2 workspace for simulation, motion planning, and GUI-based control of a 6-DOF robotic arm.
It integrates **Gazebo**, **MoveIt 2**, **ros2\_control**, **MoveIt Servo**, perception nodes, conveyor belt simulation, and a custom **Qt GUI**.

---

## ✨ Features

* **Robot Model & Description**

  * URDF/Xacro model (`parol6_description`)
  * Meshes and visuals for RViz & Gazebo
  * Integrated gripper and optional camera

* **Simulation (Gazebo)**

  * Physics-based simulation with ros2\_control
  * Conveyor belt simulation (velocity + distance services)
  * Object spawning and attach/detach services

* **Motion Planning (MoveIt 2)**

  * Move group for joint & Cartesian planning
  * Collision-aware path planning
  * KDL kinematics plugin support

* **MoveIt Servo (Jogging)**

  * Real-time jog control (X, Y, Z, Roll, Pitch, Yaw)
  * Adjustable speed scaling
  * Servo On/Off services

* **Perception Pipeline**

  * Box detection node (vision)
  * Publishes detected box pose as `PoseStamped`
  * Triggerable pick-and-place pipeline

* **Custom GUI (`parol6_gui`)**

  * **Control Tab**

    * Jog buttons: X+/−, Y+/−, Z+/−, Roll+/−, Pitch+/−, Yaw+/−
    * Servo On/Off toggle
    * Speed slider for jogging
    * Real-time joint values
  * **Targets Tab**

    * Display 6 joint values
    * Save/load joint or pose targets
    * Execute stored targets

---

## 📦 Packages in This Repo

* `parol6_description` → URDF, meshes, robot\_state\_publisher
* `parol6_moveit2_config` → MoveIt 2 configuration & bringup launch files
* `parol6_gazebo` → Gazebo worlds, conveyor belt simulation
* `parol6_gui` → Qt C++ GUI for jogging & monitoring
* `parol6_pipeline` → Perception + pick-and-place pipeline
* `msg_gazebo` → Custom attach/detach service definitions

---

## 🚀 Launch Instructions

### 1. Launch the robot, Gazebo, MoveIt 2, and RViz

This brings up the full simulation environment with controllers and planning:

```bash
ros2 launch parol6_moveit2_config bringup.launch.py
```

---

### 2. Launch the GUI

In a new terminal:

```bash
ros2 run parol6_gui parol6_gui
```

This opens the Qt GUI for jogging, servo control, and target management.

---

### 3. Example Service Calls

* Start/stop servo:

  ```bash
  ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
  ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger
  ```

* Control conveyor:

  ```bash
  ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 5.0}"
  ros2 service call /conveyor2/MoveDistance conveyorbelt_msgs/srv/MoveDistance "{distance: 0.35}"
  ```

* Spawn & attach objects:

  ```bash
  ros2 service call /spawn_box std_srvs/srv/Trigger
  ros2 service call /AttachDetach msg_gazebo/srv/AttachDetach "{model1: 'parol6', link1: 'L6', model2: 'Red_1', link2: 'link', attach: true}"
  ```

* Run pipeline & detection:

  ```bash
  ros2 run parol6_pipeline pipeline_node
  ros2 service call /start_detection std_srvs/srv/Trigger "{}"
  ros2 service call /start_picking std_srvs/srv/Trigger "{}"
  ```

---

## ⚙️ Dependencies

* ROS 2 Humble
* Gazebo 11
* MoveIt 2
* ros2\_control + ros2\_controllers
* Qt 5 / Qt 6

Install common dependencies:

```bash
sudo apt install ros-humble-moveit ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers qtbase5-dev
```

---

## 📂 Workspace Layout

```
parol6/
├── parol6_description/      # URDF, xacro, meshes
├── parol6_moveit2_config/   # MoveIt 2 config + bringup.launch.py
├── parol6_gazebo/           # Gazebo worlds, conveyor
├── parol6_gui/              # Qt C++ GUI
├── parol6_pipeline/         # Vision & pick-place pipeline
├── msg_gazebo/              # Custom Gazebo services
```

---

use exposrt gazebo model path export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/azif/ liek thsi 
