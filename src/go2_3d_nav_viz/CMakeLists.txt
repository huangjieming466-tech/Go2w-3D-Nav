cmake_minimum_required(VERSION 3.8)
project(go2_3d_nav_viz)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_executable(history_cloud_accumulator src/history_cloud_accumulator.cpp)
ament_target_dependencies(history_cloud_accumulator
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_include_directories(history_cloud_accumulator PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(history_cloud_accumulator ${PCL_LIBRARIES})

install(TARGETS history_cloud_accumulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_path_to_world_bridge.py
  scripts/mid360_utlidar_fusion.py
  scripts/pcd_map_publisher.py
  scripts/super_map_cloud_bridge.py
  scripts/super_imu_to_base_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
