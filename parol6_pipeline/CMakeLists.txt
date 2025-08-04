cmake_minimum_required(VERSION 3.8)
project(parol6_pipeline)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(box_detector_node src/box_detector.cpp)
ament_target_dependencies(box_detector_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge OpenCV
  tf2 tf2_geometry_msgs message_filters
)

# add_executable(pick_controller_node src/pick_controller.cpp)
# ament_target_dependencies(pick_controller_node rclcpp geometry_msgs std_msgs)

install(TARGETS
  box_detector_node
  # pick_controller_node
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
