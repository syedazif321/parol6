cmake_minimum_required(VERSION 3.8)
project(parol6_pipeline)

# C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ──────────────────────────────────────────────────────────────────────────────
# Dependencies
# ──────────────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# ──────────────────────────────────────────────────────────────────────────────
# Includes
# ──────────────────────────────────────────────────────────────────────────────
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ──────────────────────────────────────────────────────────────────────────────
# box_detector_node
# ──────────────────────────────────────────────────────────────────────────────
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

# ──────────────────────────────────────────────────────────────────────────────
# pick_controller_node
# ──────────────────────────────────────────────────────────────────────────────
add_executable(pick_controller_node src/pick_controller.cpp)

ament_target_dependencies(pick_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
)

# ──────────────────────────────────────────────────────────────────────────────
# Install Targets
# ──────────────────────────────────────────────────────────────────────────────
install(TARGETS
  box_detector_node
  pick_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
