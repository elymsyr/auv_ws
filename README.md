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

## FossenNet: AI-Based Controller

The core of the AUV's motion control is `FossenNet`, a deep neural network trained to imitate a computationally expensive nonlinear Model Predictive Controller (NL-MPC). This approach allows for real-time, high-performance control by leveraging a pre-trained model for inference within the ROS 2 ecosystem. The Gazebo dynamics plugin is based on Fossen's equations, and this network is trained specifically to control a vehicle governed by these dynamics.

-   **Function**: It takes the AUV's current state (velocities and orientations) and a future reference trajectory as input, and outputs the optimal commands for the 8 thrusters.
-   **Model**: The trained model is a TorchScript file (`fossen_net_scripted.pt`) located in `src/auv_control/models/`. It is loaded by the `MotionSystem` using LibTorch (the C++ distribution of PyTorch) for efficient, low-latency inference.
-   **Architecture**: `FossenNet` uses a multi-branch architecture featuring an LSTM to process the time-series trajectory data and fully connected layers to process the current state vector. The combined features are then passed through a final set of layers to predict the thruster commands. For more details on the model training and architecture, see the [auv_control_model repository](https://github.com/elymsyr/auv_control_model).

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

## License

This project is licensed under the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.

### Third-Party Content

Some of the models and assets used in the simulation are from third-party sources and are distributed under their own licenses:

-   **`herkules_ship_wreck`**: This model is based on the work from the `dave_models` repository, originally licensed under Apache 2.0.
-   **`diver`**: This model is a common Gazebo asset, and its original license should be verified if used in a derivative work.
-   **Textures and Materials**: The textures used in the `sand_heightmap`, `sea_bottom`, and `sea_surface` models are from external sources. Please refer to the `License and source for textures.txt` file located in the respective model directories for detailed information.
-   [eeUVsim_Gazebo](https://github.com/Centre-for-Biorobotics/eeUVsim_Gazebo)
