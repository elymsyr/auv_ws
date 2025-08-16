# AUV Simulation and Control Workspace

This project provides a simulation and control stack for an Autonomous Underwater Vehicle (AUV) using ROS 2 Humble and Gazebo. It features a high-fidelity dynamics model, a modular control architecture, and an MPC controller that uses a pre-trained LibTorch model.

## Key Features

-   **ROS 2 Humble Integration**: Built on ROS 2, using lifecycle nodes for robust system management.
-   **High-Fidelity Gazebo Simulation**: A custom AUV model with a dynamics plugin based on Fossen's model for realistic hydrodynamics.
-   **Lifecycle Management**: A central orchestrator node manages the startup, activation, and shutdown of all control subsystems.
-   **Advanced Control**: A Nonlinear Model Predictive Controller (MPC) uses a pre-trained LibTorch model to calculate optimal thruster commands.
-   **CUDA-Accelerated Mapping**: A 2D grid map with A* pathfinding for obstacle avoidance.

## System Architecture

-   **`auv_control`**: The main ROS 2 package containing the AUV's control systems.
    -   **Orchestrator**: A lifecycle node that manages all other systems.
    -   **Mission System**: Manages high-level mission logic (e.g., path following, station keeping).
    -   **Environment System**: Processes and converts state data from the simulation.
    -   **Motion System**: The low-level controller that uses the MPC to compute and publish thruster commands.
-   **`gazebo_sim`**: The Gazebo simulation package.
    -   **Dynamics Plugin**: Subscribes to thruster commands and applies forces to the AUV model in Gazebo.
    -   **Vehicle Model**: An implementation of the AUV's physical properties based on Fossen's model.
    -   **Worlds/Models**: Gazebo world files and the AUV's URDF.

## Installation and Build

### Prerequisites

-   ROS 2 Humble
-   Gazebo (Classic)
-   Colcon
-   NVIDIA GPU with CUDA Toolkit
-   Docker (for containerized setup)
-   Download and extract LibTorch (C++ distribution of PyTorch) into the `libtorch` directory at the workspace root.

### Docker Setup (Recommended)

1.  **Build the Docker Image**:
    ```sh
    docker build -t ros2_auv_image .
    ```

2.  **Run the Docker Container**:
    Allow local X server connections and run the container, mounting the display.
    ```sh
    xhost +
    docker run -it --rm \
        --gpus all \
        --net=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY=$DISPLAY \
        -v $(pwd):/root/auv_ws \
        ros2_auv_image
    ```

3.  **Build the Workspace (inside container)**:
    ```sh
    cd /root/auv_ws
    colcon build --symlink-install --cmake-args "-DCMAKE_PREFIX_PATH=$(pwd)/libtorch"
    ```

### Native Setup

1.  **Source ROS 2**:
    ```sh
    source /opt/ros/humble/setup.bash
    ```
2.  **Build the Workspace**:
    ```sh
    colcon build --symlink-install --cmake-args "-DCMAKE_PREFIX_PATH=$(pwd)/libtorch"
    ```

## Usage

All commands should be run from the workspace root (`auv_ws` or `/root/auv_ws` in Docker) after sourcing the setup file.

1.  **Source the Workspace**:
    ```sh
    source install/setup.bash
    ```

2.  **Launch the Simulation**:
    This starts Gazebo and the `AuvOrchestratorNode`.
    ```sh
    ros2 launch auv_control sim_with_control.launch.py
    ```

3.  **Activate the Control System**:
    In a new terminal, source the workspace again and use the `ros2 lifecycle` commands to activate the controller.
    ```sh
    # Source workspace
    source install/setup.bash

    # Configure and activate the node
    ros2 lifecycle set /auv_orchestrator configure
    ros2 lifecycle set /auv_orchestrator activate
    ```

4.  **Run a Mission**:
    In a new terminal, set and start a mission.
    ```sh
    # Source workspace
    source install/setup.bash

    # Set the mission type (0 for SonarMisTest)
    ros2 service call /mission/set auv_control/srv/SetMission "{mission_type: 0}"

    # Start the mission
    ros2 service call /mission/start std_srvs/srv/Trigger "{}"
    ```

### System Introspection

Use the following commands to inspect the running system.

```sh
# List all active nodes
ros2 node list

# List all topics and their types
ros2 topic list -t

# List all services and their types
ros2 service