cmake_minimum_required(VERSION 3.8)
project(box_spawner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(box_spawner src/box_spawner.cpp)
ament_target_dependencies(box_spawner rclcpp gazebo_msgs)
install(TARGETS box_spawner DESTINATION lib/${PROJECT_NAME})


ament_package()
