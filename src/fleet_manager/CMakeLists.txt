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

# map_merger: every robotN/map -> one /map in the world frame (placement from TF)
add_executable(map_merger src/map_merger.cpp)
ament_target_dependencies(map_merger rclcpp nav_msgs tf2_ros)

# goal_dispatcher: /goal_pose -> the robot with the shortest nav2 path
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
add_executable(goal_dispatcher src/goal_dispatcher.cpp)
ament_target_dependencies(goal_dispatcher rclcpp rclcpp_action geometry_msgs nav2_msgs)

install(TARGETS lidar_explorer gazebo_lidar map_merger goal_dispatcher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch urdf models config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/urdf_gazebo_nosensors.py DESTINATION lib/${PROJECT_NAME})

ament_package()
