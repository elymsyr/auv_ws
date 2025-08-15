cmake_minimum_required(VERSION 3.8)
project(gazebo_sim)

# =================================================================
# 1. FIND PACKAGES
# =================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(auv_control REQUIRED) # Finds the messages from your other package

# =================================================================
# 2. DEFINE TARGETS
# =================================================================
include_directories(include)

add_library(auv_dynamics_plugin SHARED
  src/auv_dynamics_plugin.cpp
  src/vehicle_model.cpp
)

# =================================================================
# 3. LINK LIBRARIES
# =================================================================
ament_target_dependencies(auv_dynamics_plugin
  rclcpp
  gazebo_ros
  std_msgs
  gazebo_msgs
  auv_control # Link against the messages
)

# Workaround for Protobuf symbol differences in Gazebo
target_compile_definitions(auv_dynamics_plugin PRIVATE
  "google::protobuf::internal::AuxillaryParseTableField=google::protobuf::internal::AuxiliaryParseTableField"
)

# =================================================================
# 4. INSTALLATION
# =================================================================
install(TARGETS
  auv_dynamics_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch urdf worlds models config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(PROGRAMS scripts/keyboard_teleop.py DESTINATION lib/${PROJECT_NAME})

ament_package()