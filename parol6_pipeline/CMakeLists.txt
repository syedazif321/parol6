cmake_minimum_required(VERSION 3.8)
project(parol6_pipeline)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(conveyorbelt_msgs REQUIRED)
find_package(msg_gazebo REQUIRED)
find_package(SQLite3 REQUIRED)



include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)


add_executable(box_detector_node src/box_detector.cpp)
ament_target_dependencies(box_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  moveit_ros_planning_interface
)

target_link_libraries(box_detector_node
  ${OpenCV_LIBS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(pick_controller_node src/pick_controller.cpp)
ament_target_dependencies(pick_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(pick_controller_node
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(analytics_logger_node src/analytics_logger_node.cpp)
ament_target_dependencies(analytics_logger_node
  rclcpp
  std_msgs
  std_srvs
  nlohmann_json
)

target_link_libraries(analytics_logger_node
  SQLite::SQLite3
)


add_executable(pipeline_node src/pipeline.cpp)
ament_target_dependencies(pipeline_node
  rclcpp
  std_srvs
  conveyorbelt_msgs
  msg_gazebo
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  nlohmann_json
)

if(conveyorbelt_msgs_EXPORTED_TARGETS)
  add_dependencies(pipeline_node ${conveyorbelt_msgs_EXPORTED_TARGETS})
endif()

if(msg_gazebo_EXPORTED_TARGETS)
  add_dependencies(pipeline_node ${msg_gazebo_EXPORTED_TARGETS})
endif()

install(TARGETS
  box_detector_node
  pick_controller_node
  pipeline_node
  analytics_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()