cmake_minimum_required(VERSION 3.8)
project(gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === FIND DEPENDENCIES ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# === DEFINE AND BUILD YOUR C++ PLUGIN ===
add_library(auv_dynamics_plugin SHARED
  src/auv_dynamics_plugin.cpp
  src/vehicle_model.cpp
)

target_compile_definitions(auv_dynamics_plugin
  PRIVATE
    "google::protobuf::internal::AuxillaryParseTableField=google::protobuf::internal::AuxiliaryParseTableField"
)

# === CONFIGURE COMPILER AND LINKER ===

# THIS IS THE CORRECTED SECTION
# Add Gazebo's required compiler flags (e.g., -std=c++17) to your target
target_compile_options(auv_dynamics_plugin PRIVATE ${GAZEBO_CXX_FLAGS})

# Add Gazebo's include directories to your target's search path
target_include_directories(auv_dynamics_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GAZEBO_INCLUDE_DIRS}
)

# Link against ROS 2 libraries
ament_target_dependencies(auv_dynamics_plugin
  rclcpp
  std_msgs
  gazebo_ros
  gazebo_msgs
)

# Link against Eigen and all of Gazebo's required libraries
target_link_libraries(auv_dynamics_plugin
  Eigen3::Eigen
  ${GAZEBO_LIBRARIES}
)

# === INSTALLATION RULES ===
install(
  TARGETS auv_dynamics_plugin
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch urdf worlds config models
  DESTINATION share/${PROJECT_NAME}/
)
install(
  PROGRAMS scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()