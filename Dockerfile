FROM nvidia/cuda:12.1.1-devel-ubuntu22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git curl wget gnupg2 lsb-release ca-certificates locales build-essential cmake python3-pip unzip && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    libeigen3-dev \
    nlohmann-json3-dev && \
    rm -rf /var/lib/apt/lists/*

RUN wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.3.1%2Bcu121.zip -O libtorch.zip && \
    unzip libtorch.zip && \
    mv libtorch /opt/libtorch && \
    rm libtorch.zip

ENV Torch_DIR=/opt/libtorch/share/cmake/Torch

WORKDIR /ros2_ws

COPY ./src ./src

RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "Eigen3 nlohmann_json"

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DTORCH_CUDA_ARCH_LIST="7.5;8.0;8.6;9.0"

RUN echo '#!/bin/bash' >> /docker_entrypoint.sh && \
    echo 'set -e' >> /docker_entrypoint.sh && \
    echo '' >> /docker_entrypoint.sh && \
    echo '# Source ROS 2 setup' >> /docker_entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /docker_entrypoint.sh && \
    echo '' >> /docker_entrypoint.sh && \
    echo '# Source workspace setup' >> /docker_entrypoint.sh && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /docker_entrypoint.sh && \
    echo '' >> /docker_entrypoint.sh && \
    echo '# Add LibTorch to the library path so executables can find it at runtime' >> /docker_entrypoint.sh && \
    echo 'export LD_LIBRARY_PATH=/opt/libtorch/lib:$LD_LIBRARY_PATH' >> /docker_entrypoint.sh && \
    echo '' >> /docker_entrypoint.sh && \
    echo '# Execute the command passed to the container' >> /docker_entrypoint.sh && \
    echo 'exec "$@"' >> /docker_entrypoint.sh && \
    chmod +x /docker_entrypoint.sh

ENTRYPOINT ["/docker_entrypoint.sh"]

CMD ["bash"]