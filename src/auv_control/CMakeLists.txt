cmake_minimum_required(VERSION 3.8)
project(auv_control LANGUAGES C CXX CUDA)

# =================================================================
# 1. BOILERPLATE AND COMPILER SETTINGS
# =================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CUDA_ARCHITECTURES 75 80 86 90)

set(CMAKE_PREFIX_PATH /home/elymsyr/GitHub/robot_sim/libtorch)

# =================================================================
# 2. FIND ALL REQUIRED PACKAGES
# =================================================================
find_package(Torch REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_py REQUIRED) 
find_package(builtin_interfaces REQUIRED)
find_package(CUDA REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# =================================================================
# 3. GENERATE ROS 2 INTERFACES (.msg/.srv)
# =================================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandTopic.msg" "msg/EnvironmentTopic.msg" "msg/MissionTopic.msg"
  "msg/MotionTopic.msg"  "msg/SignalTopic.msg"    "msg/StateTopic.msg"
  "msg/TestSonarTopic.msg" "msg/TrajectoryPoint.msg"
  "srv/GetStatus.srv" "srv/SetMission.srv" "srv/Trigger.srv"
  DEPENDENCIES builtin_interfaces rosidl_generator_py std_msgs
)

# =================================================================
# 3.5: CRITICAL FIX - Set linker language for all ROS IDL targets
# =================================================================
# Must be placed IMMEDIATELY after rosidl_generate_interfaces
set(rosidl_targets
  "${PROJECT_NAME}__rosidl_generator_c"
  "${PROJECT_NAME}__rosidl_typesupport_introspection_c"
  "${PROJECT_NAME}__rosidl_typesupport_c"
  "${PROJECT_NAME}__rosidl_generator_cpp"
  "${PROJECT_NAME}__rosidl_typesupport_introspection_cpp"
  "${PROJECT_NAME}__rosidl_typesupport_cpp"
  "${PROJECT_NAME}__rosidl_generator_py"
  "${PROJECT_NAME}__rosidl_typesupport_fastrtps_c__pyext"
  "${PROJECT_NAME}__rosidl_typesupport_introspection_c__pyext"
  "${PROJECT_NAME}__rosidl_typesupport_c__pyext"
)

foreach(target ${rosidl_targets})
  if(TARGET ${target})
    # use C linkage for the "typesupport_c" and python extension targets so the PyInit_... symbol is unmangled
    if("${target}" MATCHES "typesupport_c" OR "${target}" MATCHES "__pyext$")
      set_target_properties(${target} PROPERTIES LINKER_LANGUAGE C)
    else()
      # keep C++ linkage for C++ typesupport targets
      set_target_properties(${target} PROPERTIES LINKER_LANGUAGE CXX)
    endif()
  endif()
endforeach()

# =================================================================
# 4. DEFINE AND COMPILE SOURCES
# =================================================================
include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
)

set(CUDA_SOURCES
  src/mapping/environment_astar.cu
  src/mapping/environment_global.cu
  src/mapping/environment_map.cu
)
add_library(mapping_cuda_objects OBJECT ${CUDA_SOURCES})
set_target_properties(mapping_cuda_objects PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(CPP_SOURCES
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
add_executable(auv_orchestrator_node ${CPP_SOURCES})

# =================================================================
# 5. LINK LIBRARIES AND DEPENDENCIES
# =================================================================
# Use modern linking approach instead of deprecated function
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(auv_orchestrator_node
  ${typesupport_target}
  mapping_cuda_objects
  nlohmann_json::nlohmann_json
  ${CUDA_LIBRARIES}
  ${TORCH_LIBRARIES}
)

ament_target_dependencies(auv_orchestrator_node
  rclcpp
  rclcpp_lifecycle
  builtin_interfaces
  gazebo_msgs
  std_msgs
  gazebo_msgs
)

# =================================================================
# 6. INSTALLATION RULES
# =================================================================
install(TARGETS
  auv_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# =================================================================
# 7. FINAL AMENT PACKAGE CALL
# =================================================================
ament_package()