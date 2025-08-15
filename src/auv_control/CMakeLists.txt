cmake_minimum_required(VERSION 3.8)
project(auv_control LANGUAGES C CXX CUDA)

# =================================================================
# 1. COMPILER SETTINGS
# =================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

set(CMAKE_CUDA_ARCHITECTURES 75 80 86 90)

# =================================================================
# 2. FIND PACKAGES
# =================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Torch REQUIRED)
find_package(CUDA REQUIRED)
find_package(gazebo_msgs REQUIRED)

# =================================================================
# 3. GENERATE ROS 2 INTERFACES
# =================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandTopic.msg"
  "msg/EnvironmentTopic.msg"
  "msg/MissionTopic.msg"
  "msg/MotionTopic.msg"
  "msg/SignalTopic.msg"
  "msg/StateTopic.msg"
  "msg/TestSonarTopic.msg"
  "msg/TrajectoryPoint.msg"
  "srv/GetStatus.srv"
  "srv/SetMission.srv"
  "srv/Trigger.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# =================================================================
# 4. DEFINE SOURCES AND TARGETS
# =================================================================
include_directories(include ${CUDA_INCLUDE_DIRS})

# --- CUDA Object Library for Mapping ---
add_library(mapping_cuda_objects OBJECT
  src/mapping/environment_astar.cu
  src/mapping/environment_global.cu
  src/mapping/environment_map.cu
)
set_target_properties(mapping_cuda_objects PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- Main ROS 2 Orchestrator Node ---
add_executable(auv_orchestrator_node
  src/system/control_system.cpp
  src/system/environment_system.cpp
  src/system/auv_orchestrator_node.cpp
  src/system/mission_system.cpp
  src/system/motion_system.cpp
  src/control/fossennet.cpp
  src/control/model_inference.cpp
  src/mapping/environment_helper.cpp
  src/mission/mission_sonar_imp.cpp
)

# =================================================================
# 5. LINK LIBRARIES
# =================================================================
rosidl_get_typesupport_target(idl_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(auv_orchestrator_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  builtin_interfaces
  gazebo_msgs
)

target_link_libraries(auv_orchestrator_node
  ${idl_typesupport_target}
  $<TARGET_OBJECTS:mapping_cuda_objects>
  nlohmann_json::nlohmann_json
  ${CUDA_LIBRARIES}
  ${TORCH_LIBRARIES}
  Eigen3::Eigen
)

# =================================================================
# 6. INSTALLATION
# =================================================================
install(TARGETS
  auv_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME} OPTIONAL)

# You moved keyboard_teleop.py to gazebo_sim, so it should not be here.

ament_package()