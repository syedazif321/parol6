# parol6
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/azif/projetcs/parol6/install/parol6_description/share
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/azif/projetcs/parol6/install/parol6_description/share/parol6_description


ros2 topic pub -r 10 /servo_node/delta_twist_cmds geometry_msgs/TwistStamped -- \
"header:
  stamp:
    sec: $(date +%s)
    nanosec: 0
  frame_id: 'base_link'
twist:
  linear:
    x: 0.1
    y: 0.1
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"



ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger

ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl power:\ 05.0\ 

ros2 service call /conveyor2/MoveDistance conveyorbelt_msgs/srv/MoveDistance "{distance: 0.35}"

ros2 service call /spawn_box std_srvs/srv/Trigger

ros2 service call /AttachDetach msg_gazebo/srv/AttachDetach \
  "{model1: 'parol6', link1: 'L6', model2: 'Red_1', link2: 'link', attach: true}"



ros2 param set /move_group arm.kinematics_solver kdl_kinematics_plugin/KDLKinematicsPlugin

ros2 param set /move_group arm.kinematics_solver_timeout 0.05
ros2 param set /move_group arm.kinematics_solver_attempts 5
ros2 param set /move_group arm.kinematics_solver_search_resolution 0.005

azif@azif:~/projetcs/parol6$ ros2 topic echo /detected_box_pose --once 
header:
  stamp:
    sec: 1754561418
    nanosec: 84726035
  frame_id: base_link
pose:
  position:
    x: 0.5422454328699826
    y: 0.07943160904603358
    z: 0.020987296679036538
  orientation:
    x: 0.9850946354014032
    y: -0.17201140999115824
    z: -0.0007843480920685094
    w: 0.0001376019647234278



      pose: {
    position: {
      x: 0.4419158783051373,
      y: 0.08867201398535185,
      z: 0.1957374591062092
    },
    orientation: {
      x: 0.7096465625406891,
      y: 0.00081043727815465,
      z: -0.7045544569662532,
      w: 0.00202894913455497
    }
  }


        "box1": [
        0.19802173457379446,
        0.8001236248778865,
        0.060721036579216126,
        -0.25195117499927466,
        -0.88404622879548,
        -0.16494299953626967


}"





ros2 topic pub /detected_box_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.4,y: 0.2, z: 0.2}, orientation: {x: 1.0, y: 0.5, z: -4.0, w: 1.0}}}


ros2 topic pub /detected_box_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.4, y: 0.0, z: 0.2},
    orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}
  }
}"



