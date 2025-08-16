# AUV Simulation and Control Workspace

This project provides a complete simulation and control stack for an Autonomous Underwater Vehicle (AUV) using ROS 2, Gazebo, and a Model Predictive Controller (MPC) with a deep learning component. It is a full-featured ROS 2 implementation designed for simulating advanced AUV behaviors in a realistic environment.

## Overview

The system is designed as a modular ROS 2 application built around lifecycle nodes for robust management. A central orchestrator node manages several subsystems responsible for mission planning, state estimation, and motion control. The AUV's dynamics are simulated in Gazebo using a custom plugin that implements Fossen's model of marine craft dynamics, providing a high-fidelity simulation environment.

## Key Features

-   **ROS 2 Humble Integration**: The entire software stack is built on ROS 2 Humble, utilizing modern features like lifecycle nodes and component-based architecture.
-   **High-Fidelity Gazebo Simulation**: A realistic underwater world with a custom AUV model. The simulation includes:
    -   A dynamics plugin based on Fossen's model, calculating hydrodynamics, damping, and restoring forces.
    -   Custom GLSL shaders for realistic water surface and seabed rendering.
-   **Lifecycle Management**: The core control node ([`AuvOrchestratorNode`](src/auv_control/include/auv_control/system/auv_orchestrator_node.hpp)) is a lifecycle node, ensuring that all subsystems are configured, activated, and deactivated in a controlled and predictable manner.
-   **Advanced Control**: A Nonlinear Model Predictive Controller ([`NonlinearMPC`](src/auv_control/include/auv_control/control/fossennet.h)) uses a pre-trained PyTorch/LibTorch model to predict the AUV's future states and determine optimal thruster commands.
-   **CUDA-Accelerated Mapping**: The [`EnvironmentMap`](src/auv_control/include/auv_control/mapping/environment.h) uses CUDA for high-performance 2D grid mapping, obstacle avoidance, and A* pathfinding.
-   **Modular Mission System**: A flexible mission system allows for defining and executing complex behaviors, such as the included [`SonarMission`](src/auv_control/include/auv_control/mission/mission_sonar_imp.h) for obstacle avoidance.

## Workspace Structure

-   `src/`: Contains the source code for the ROS 2 packages.
    -   `auv_control`: The primary ROS 2 package containing the vehicle's control and intelligence systems.
    -   `gazebo_sim`: The Gazebo simulation package, including the AUV model, world files, and dynamics plugin.
-   `build/`: The build directory where `colcon` compiles the code.
-   `install/`: The installation directory where compiled targets and launch files are placed.
-   `log/`: Contains logs from ROS 2 nodes.
-   `libtorch/`: The LibTorch (PyTorch C++) library, required by the `auv_control` package.

## System Architecture

The system is composed of two main ROS 2 packages that work in concert.

### 1. AUV Control (`auv_control`)

This package is the brain of the AUV. It runs as a single executable (`auv_orchestrator_node`) containing multiple logical systems managed by a lifecycle orchestrator.

-   **Orchestrator (`AuvOrchestratorNode`)**: A ROS 2 lifecycle node that manages the startup, shutdown, and state transitions of all other systems. Found in [`src/auv_control/include/auv_control/system/auv_orchestrator_node.hpp`](src/auv_control/include/auv_control/system/auv_orchestrator_node.hpp).

-   **Mission System (`MissionSystem`)**: Manages high-level mission logic. It can load, start, and stop different missions (e.g., [`SonarMission`](src/auv_control/include/auv_control/mission/mission_sonar_imp.h)). It exposes ROS 2 services (`/mission/set`, `/mission/start`, `/mission/stop`) for external control and publishes the desired trajectory on the `/mission_trajectory` topic.
    -   Implementation: [`src/auv_control/src/system/mission_system.cpp`](src/auv_control/src/system/mission_system.cpp)

-   **Environment System (`EnvironmentSystem`)**: Subscribes to the raw state data from the Gazebo simulation (`/ucat/state`), converts it from Gazebo's ENU (East-North-Up) frame to the standard NED (North-East-Down) frame used by the controller, and publishes it on the `/environment_state` topic.
    -   Implementation: [`src/auv_control/src/system/environment_system.cpp`](src/auv_control/src/system/environment_system.cpp)

-   **Motion System (`MotionSystem`)**: The low-level controller. It receives the desired trajectory from the `MissionSystem` and the current state from the `EnvironmentSystem`. It uses the `NonlinearMPC` to calculate the required thruster forces and publishes them as a `std_msgs::msg::Float32MultiArray` to the `/ucat/thruster_cmd` topic for the Gazebo plugin to consume.
    -   Header: [`src/auv_control/include/auv_control/system/motion_system.h`](src/auv_control/include/auv_control/system/motion_system.h)

-   **Nonlinear MPC (`NonlinearMPC`)**: A controller that uses a pre-trained PyTorch model via the [`ModelInference`](src/auv_control/include/auv_control/control/model_inference.h) class to predict the AUV's future states and determine the optimal thruster commands.
    -   Header: [`src/auv_control/include/auv_control/control/fossennet.h`](src/auv_control/include/auv_control/control/fossennet.h)

-   **Mapping (`EnvironmentMap`)**: A CUDA-accelerated 2D grid map used for obstacle avoidance and pathfinding with an A* algorithm. It features a sliding window to keep the map centered on the AUV.
    -   Header: [`src/auv_control/include/auv_control/mapping/environment.h`](src/auv_control/include/auv_control/mapping/environment.h)

### 2. Gazebo Simulation (`gazebo_sim`)

This package contains everything needed to simulate the AUV.

-   **Dynamics Plugin (`AuvDynamicsPlugin`)**: A Gazebo `ModelPlugin` that applies forces and torques to the AUV model. It subscribes to the `/ucat/thruster_cmd` topic, simulates the vehicle's hydrodynamics using the [`VehicleModel`](src/gazebo_sim/include/gazebo_sim/vehicle_model.h), and publishes the ground-truth state as a `gazebo_msgs::msg::EntityState` on the `/ucat/state` topic.
    -   Implementation: [`src/gazebo_sim/src/auv_dynamics_plugin.cpp`](src/gazebo_sim/src/auv_dynamics_plugin.cpp)

-   **Vehicle Model (`VehicleModel`)**: A C++ implementation of the AUV's physical properties (mass, inertia, damping, thruster allocation matrix), loaded from a configuration file. It implements Fossen's model, including methods for calculating the [`coriolis_matrix`](src/gazebo_sim/src/vehicle_model.cpp), [`damping_matrix`](src/gazebo_sim/src/vehicle_model.cpp), and [`restoring_forces`](src/gazebo_sim/src/vehicle_model.cpp).
    -   Implementation: [`src/gazebo_sim/src/vehicle_model.cpp`](src/gazebo_sim/src/vehicle_model.cpp)

-   **URDF & Worlds**: Contains the AUV's visual and collision models (`.xacro`, `.stl`, `.dae`), Gazebo world files (`.world`), and custom GLSL shaders for water effects.

-   **Teleoperation**: A keyboard teleop script ([`keyboard_teleop.py`](src/gazebo_sim/scripts/keyboard_teleop.py)) is provided for manual control of the AUV by publishing directly to the thruster command topic.

## Building the Workspace

**Prerequisites:**
-   ROS 2 Humble
-   Colcon
-   Gazebo (Classic)
-   LibTorch (C++ distribution of PyTorch, placed in the `libtorch` directory)
-   CUDA Toolkit (for `auv_control` mapping)

**Build Steps:**
1.  Ensure you have sourced your ROS 2 installation:
    ```sh
    source /opt/ros/humble/setup.bash
    ```
2.  Navigate to the root of the workspace (`auv_ws`).
3.  Run the build command:
    ```sh
    colcon build --symlink-install
    ```

## Running the Simulation

1.  Source the local workspace setup script from the workspace root:
    ```sh
    source install/local_setup.bash
    ```
2.  Launch the simulation with the controller using the provided launch file:
    ```sh
    ros2 launch auv_control sim_with_control.launch.py
    ```
    This will start the Gazebo server and client, spawn the AUV, and launch the `AuvOrchestratorNode`. The orchestrator will then automatically bring up the control system.

## Interacting with the System

You can control the mission using ROS 2 services from the command line.

-   **Set a Mission** (e.g., `SonarMisTest`, which has `mission_type` enum value 0):
    ```sh
    ros2 service call /mission/set auv_control/srv/SetMission "{mission_type: 0}"
    ```

-   **Start the Active Mission**:
    ```sh
    ros2 service call /mission/start std_srvs/srv/Trigger
    ```

-   **Stop the Active Mission**:
    ```sh
    ros2 service call /mission/stop std_srvs/srv/Trigger
    ```

-   **Reset the Simulation**:
    ```sh
    ros2 topic pub /ucat/reset std_msgs/msg/Bool "data: true" -1
    ```
<!-- filepath: README.md -->
# AUV Simulation and Control Workspace

This project provides a complete simulation and control stack for an Autonomous Underwater Vehicle (AUV) using ROS 2, Gazebo, and a Model Predictive Controller (MPC) with a deep learning component. It is a full-featured ROS 2 implementation designed for simulating advanced AUV behaviors in a realistic environment.

## Overview

The system is designed as a modular ROS 2 application built around lifecycle nodes for robust management. A central orchestrator node manages several subsystems responsible for mission planning, state estimation, and motion control. The AUV's dynamics are simulated in Gazebo using a custom plugin that implements Fossen's model of marine craft dynamics, providing a high-fidelity simulation environment.

## Key Features

-   **ROS 2 Humble Integration**: The entire software stack is built on ROS 2 Humble, utilizing modern features like lifecycle nodes and component-based architecture.
-   **High-Fidelity Gazebo Simulation**: A realistic underwater world with a custom AUV model. The simulation includes:
    -   A dynamics plugin based on Fossen's model, calculating hydrodynamics, damping, and restoring forces.
    -   Custom GLSL shaders for realistic water surface and seabed rendering.
-   **Lifecycle Management**: The core control node ([`AuvOrchestratorNode`](src/auv_control/include/auv_control/system/auv_orchestrator_node.hpp)) is a lifecycle node, ensuring that all subsystems are configured, activated, and deactivated in a controlled and predictable manner.
-   **Advanced Control**: A Nonlinear Model Predictive Controller ([`NonlinearMPC`](src/auv_control/include/auv_control/control/fossennet.h)) uses a pre-trained PyTorch/LibTorch model to predict the AUV's future states and determine optimal thruster commands.
-   **CUDA-Accelerated Mapping**: The [`EnvironmentMap`](src/auv_control/include/auv_control/mapping/environment.h) uses CUDA for high-performance 2D grid mapping, obstacle avoidance, and A* pathfinding.
-   **Modular Mission System**: A flexible mission system allows for defining and executing complex behaviors, such as the included [`SonarMission`](src/auv_control/include/auv_control/mission/mission_sonar_imp.h) for obstacle avoidance.

## Workspace Structure

-   `src/`: Contains the source code for the ROS 2 packages.
    -   `auv_control`: The primary ROS 2 package containing the vehicle's control and intelligence systems.
    -   `gazebo_sim`: The Gazebo simulation package, including the AUV model, world files, and dynamics plugin.
-   `build/`: The build directory where `colcon` compiles the code.
-   `install/`: The installation directory where compiled targets and launch files are placed.
-   `log/`: Contains logs from ROS 2 nodes.
-   `libtorch/`: The LibTorch (PyTorch C++) library, required by the `auv_control` package.

## System Architecture

The system is composed of two main ROS 2 packages that work in concert.

### 1. AUV Control (`auv_control`)

This package is the brain of the AUV. It runs as a single executable (`auv_orchestrator_node`) containing multiple logical systems managed by a lifecycle orchestrator.

-   **Orchestrator (`AuvOrchestratorNode`)**: A ROS 2 lifecycle node that manages the startup, shutdown, and state transitions of all other systems. Found in [`src/auv_control/include/auv_control/system/auv_orchestrator_node.hpp`](src/auv_control/include/auv_control/system/auv_orchestrator_node.hpp).

-   **Mission System (`MissionSystem`)**: Manages high-level mission logic. It can load, start, and stop different missions (e.g., [`SonarMission`](src/auv_control/include/auv_control/mission/mission_sonar_imp.h)). It exposes ROS 2 services (`/mission/set`, `/mission/start`, `/mission/stop`) for external control and publishes the desired trajectory on the `/mission_trajectory` topic.
    -   Implementation: [`src/auv_control/src/system/mission_system.cpp`](src/auv_control/src/system/mission_system.cpp)

-   **Environment System (`EnvironmentSystem`)**: Subscribes to the raw state data from the Gazebo simulation (`/ucat/state`), converts it from Gazebo's ENU (East-North-Up) frame to the standard NED (North-East-Down) frame used by the controller, and publishes it on the `/environment_state` topic.
    -   Implementation: [`src/auv_control/src/system/environment_system.cpp`](src/auv_control/src/system/environment_system.cpp)

-   **Motion System (`MotionSystem`)**: The low-level controller. It receives the desired trajectory from the `MissionSystem` and the current state from the `EnvironmentSystem`. It uses the `NonlinearMPC` to calculate the required thruster forces and publishes them as a `std_msgs::msg::Float32MultiArray` to the `/ucat/thruster_cmd` topic for the Gazebo plugin to consume.
    -   Header: [`src/auv_control/include/auv_control/system/motion_system.h`](src/auv_control/include/auv_control/system/motion_system.h)

-   **Nonlinear MPC (`NonlinearMPC`)**: A controller that uses a pre-trained PyTorch model via the [`ModelInference`](src/auv_control/include/auv_control/control/model_inference.h) class to predict the AUV's future states and determine the optimal thruster commands.
    -   Header: [`src/auv_control/include/auv_control/control/fossennet.h`](src/auv_control/include/auv_control/control/fossennet.h)

-   **Mapping (`EnvironmentMap`)**: A CUDA-accelerated 2D grid map used for obstacle avoidance and pathfinding with an A* algorithm. It features a sliding window to keep the map centered on the AUV.
    -   Header: [`src/auv_control/include/auv_control/mapping/environment.h`](src/auv_control/include/auv_control/mapping/environment.h)

### 2. Gazebo Simulation (`gazebo_sim`)

This package contains everything needed to simulate the AUV.

-   **Dynamics Plugin (`AuvDynamicsPlugin`)**: A Gazebo `ModelPlugin` that applies forces and torques to the AUV model. It subscribes to the `/ucat/thruster_cmd` topic, simulates the vehicle's hydrodynamics using the [`VehicleModel`](src/gazebo_sim/include/gazebo_sim/vehicle_model.h), and publishes the ground-truth state as a `gazebo_msgs::msg::EntityState` on the `/ucat/state` topic.
    -   Implementation: [`src/gazebo_sim/src/auv_dynamics_plugin.cpp`](src/gazebo_sim/src/auv_dynamics_plugin.cpp)

-   **Vehicle Model (`VehicleModel`)**: A C++ implementation of the AUV's physical properties (mass, inertia, damping, thruster allocation matrix), loaded from a configuration file. It implements Fossen's model, including methods for calculating the [`coriolis_matrix`](src/gazebo_sim/src/vehicle_model.cpp), [`damping_matrix`](src/gazebo_sim/src/vehicle_model.cpp), and [`restoring_forces`](src/gazebo_sim/src/vehicle_model.cpp).
    -   Implementation: [`src/gazebo_sim/src/vehicle_model.cpp`](src/gazebo_sim/src/vehicle_model.cpp)

-   **URDF & Worlds**: Contains the AUV's visual and collision models (`.xacro`, `.stl`, `.dae`), Gazebo world files (`.world`), and custom GLSL shaders for water effects.

-   **Teleoperation**: A keyboard teleop script ([`keyboard_teleop.py`](src/gazebo_sim/scripts/keyboard_teleop.py)) is provided for manual control of the AUV by publishing directly to the thruster command topic.

## Building the Workspace

**Prerequisites:**
-   ROS 2 Humble
-   Colcon
-   Gazebo (Classic)
-   LibTorch (C++ distribution of PyTorch, placed in the `libtorch` directory)
-   CUDA Toolkit (for `auv_control` mapping)

**Build Steps:**
1.  Ensure you have sourced your ROS 2 installation:
    ```sh
    source /opt/ros/humble/setup.bash
    ```
2.  Navigate to the root of the workspace (`auv_ws`).
3.  Run the build command:
    ```sh
    colcon build --symlink-install
    ```

## Running the Simulation

1.  Source the local workspace setup script from the workspace root:
    ```sh
    source install/local_setup.bash
    ```
2.  Launch the simulation with the controller using the provided launch file:
    ```sh
    ros2 launch auv_control sim_with_control.launch.py
    ```
    This will start the Gazebo server and client, spawn the AUV, and launch the `AuvOrchestratorNode`. The orchestrator will then automatically bring up the control system.

## Interacting with the System

You can control the mission using ROS 2 services from the command line.

-   **Set a Mission** (e.g., `SonarMisTest`, which has `mission_type` enum value 0):
    ```sh
    ros2 service call /mission/set auv_control/srv/SetMission "{mission_type: 0}"
    ```

-   **Start the Active Mission**:
    ```sh
    ros2 service call /mission/start std_srvs/srv/Trigger
    ```

-   **Stop the Active Mission**:
    ```sh
    ros2 service call /mission/stop std_srvs/srv/Trigger
    ```

-   **Reset the Simulation**:
    ```sh
    ros2 topic