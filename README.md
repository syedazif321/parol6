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

