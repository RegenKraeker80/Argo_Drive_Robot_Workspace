cmake_minimum_required(VERSION 3.5)
project(ros2_ws_robot_launch)

# Abhängigkeiten finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ros2_control REQUIRED)
find_package(soem REQUIRED)
# EtherCAT manuell hinzufügen (da es nicht als ROS-Paket existiert)
set(EtherCAT_DIR ~/ethercat)
find_library(EtherCAT_LIB NAMES ethercat PATHS ~/ethercat/lib /usr/local/lib /usr/lib)
include_directories(${EtherCAT_DIR}/include)

# Include-Pfade setzen
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${SOEM_INCLUDE_DIRS})

# EtherCAT Nodes
add_executable(ethercat_node_slave1 ethercat/ethercat_node_slave1.cpp)
ament_target_dependencies(ethercat_node_slave1 rclcpp soem std_msgs geometry_msgs)

add_executable(ethercat_logic_node ethercat/ethercat_logic_node.cpp)
ament_target_dependencies(ethercat_logic_node rclcpp soem std_msgs)

add_executable(ethercat_read_slave1 ethercat/ethercat_read_slave1.cpp)
ament_target_dependencies(ethercat_read_slave1 rclcpp soem std_msgs geometry_msgs nav_msgs tf2)

# ROS2 Control Hardware Plugin
add_library(argo_drive_hardware SHARED ethercat/argo_drive_hardware.cpp)
ament_target_dependencies(argo_drive_hardware 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  tf2 
  soem
  hardware_interface
  pluginlib
  controller_interface
  ros2_control
)

# EtherCAT als manuelle Bibliothek einfügen
target_link_libraries(argo_drive_hardware ${EtherCAT_LIB})

# ROS2 Scan Merger Node
add_executable(scan_merger scripts/scan_merger.cpp)
ament_target_dependencies(scan_merger 
  rclcpp 
  std_msgs 
  sensor_msgs
  pcl_conversions
  rcl_interfaces
  rcutils
  tf2_geometry_msgs
  geometry_msgs
)

# Installationsregeln
install(TARGETS
  argo_drive_hardware
  ethercat_read_slave1
  ethercat_node_slave1
  ethercat_logic_node
  scan_merger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launcher DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/scan_merger.py
  scripts/scan_merger.cpp
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface ethercat/plugin.xml)
ament_export_libraries(argo_drive_hardware)

install(FILES ethercat/plugin.xml DESTINATION share/${PROJECT_NAME}/ethercat)
set_target_properties(argo_drive_hardware PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_package()
