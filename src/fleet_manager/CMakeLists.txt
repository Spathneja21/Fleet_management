cmake_minimum_required(VERSION 3.8)
project(fleet_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# lidar_explorer: frontier exploration on the map + lidar gap steering -> cmd_vel
add_executable(lidar_explorer src/lidar_explorer.cpp)
ament_target_dependencies(lidar_explorer rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)

# gazebo_lidar: scan by ray casting against the world's collision boxes (Gazebo's gpu_lidar crashes here)
add_executable(gazebo_lidar src/gazebo_lidar.cpp)
ament_target_dependencies(gazebo_lidar rclcpp sensor_msgs nav_msgs)

install(TARGETS lidar_explorer gazebo_lidar DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch urdf models config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/urdf_gazebo_nosensors.py DESTINATION lib/${PROJECT_NAME})

ament_package()
