# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

# Collect the arguments for the base ros image
ARG ROS_VERSION=humble
ARG UBUNTU_VERSION=jammy

FROM ros:${ROS_VERSION}-ros-base-${UBUNTU_VERSION}

ARG ROS_VERSION
ARG UBUNTU_VERSION

# Argument to control the type of docker image to build
ARG IMAGE_TYPE=full  # Default to 'full'

# Arguments for the source repos 
ARG O3DE_REPO=https://github.com/o3de/o3de.git
ARG O3DE_BRANCH=development

ARG O3DE_EXTRAS_REPO=https://github.com/o3de/o3de-extras.git
ARG O3DE_EXTRAS_BRANCH=development

ARG ROSCON_DEMO_REPO=https://github.com/o3de/ROSConDemo.git
ARG ROSCON_DEMO_BRANCH=main

# Additional argument to control build concurrency
ARG CMAKE_JOBS=8
ARG DEBIAN_FRONTEND=noninteractive

ENV WORKSPACE=/data/workspace

WORKDIR $WORKSPACE

# Verify only the supported ROS2 versions was specified
RUN if [ "${ROS_VERSION}" = "humble" ]; then \
        echo "Building ROS2/Humble based image"; \
    elif [ "${ROS_VERSION}" = "galactic" ]; then \
        echo "Building ROS2/Galactic based image"; \
    else \
        echo "Unsupported ROS_VERSION: ${ROS_VERSION}" && exit 1; \
    fi

# Setup time zone and locale data (necessary for SSL and HTTPS packages)
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get -y \
    install \
    tzdata \
    locales \
    keyboard-configuration \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && update-locale LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Install the required ubuntu packages
RUN apt-get update && apt-get install -y \
    bc \
    bind9-utils \
    binutils \
    ca-certificates \
    clang \
    cmake \
    curl \
    file \
    firewalld \
    git \
    git-lfs \
    jq \
    kbd \
    kmod \
    less \
    lsb-release \
    libglu1-mesa-dev \
    libxcb-xinerama0 \
    libfontconfig1-dev \
    libcurl4-openssl-dev \
    libnvidia-gl-470 \
    libssl-dev \
    libxcb-xkb-dev \
    libxkbcommon-x11-dev \
    libxkbcommon-dev \
    libxcb-xfixes0-dev \
    libxcb-xinput-dev \
    libxcb-xinput0 \
    libpcre2-16-0 \
    lsof \
    net-tools \
    ninja-build \
    pciutils \
    python3-pip \
    software-properties-common \
    sudo \
    tar \
    unzip \
    vim \
    wget \
    xz-utils \
    && rm -rf /var/lib/apt/lists/*

# Setup and install the ROS packages
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-urdfdom \
    ros-${ROS_DISTRO}-vision-msgs \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-cyclonedds \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-topic-tools \
    && rm -rf /var/lib/apt/lists/* \
    && pip install python-statemachine

# Depending on the image type, sync, setup and build only whats needed
RUN if [ "${IMAGE_TYPE}" = "full" ]; then \
        git clone --recursive $O3DE_REPO $WORKSPACE/o3de \
        && git -C $WORKSPACE/o3de checkout $O3DE_BRANCH \
        && git -C $WORKSPACE/o3de lfs install \
        && git -C $WORKSPACE/o3de lfs pull \
        && $WORKSPACE/o3de/python/get_python.sh \
        && $WORKSPACE/o3de/scripts/o3de.sh register -ep $WORKSPACE/o3de \
        && git clone $O3DE_EXTRAS_REPO \
        && git -C $WORKSPACE/o3de-extras checkout $O3DE_EXTRAS_BRANCH \
        && $WORKSPACE/o3de/scripts/o3de.sh register -gp $WORKSPACE/o3de-extras/Gems/ROS2 \
        && git clone $ROSCON_DEMO_REPO \
        && git -C $WORKSPACE/ROSConDemo checkout $ROSCON_DEMO_BRANCH \
        && git -C $WORKSPACE/ROSConDemo lfs install \
        && git -C $WORKSPACE/ROSConDemo lfs pull \
        && $WORKSPACE/o3de/scripts/o3de.sh register -pp $WORKSPACE/ROSConDemo/Project \
        && . /opt/ros/${ROS_DISTRO}/setup.sh \
        && cd $WORKSPACE/ROSConDemo/Project \
        && cmake -B $WORKSPACE/ROSConDemo/Project/build/linux -S . -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DLY_STRIP_DEBUG_SYMBOLS=ON \
        && cmake --build $WORKSPACE/ROSConDemo/Project/build/linux --config profile --target ROSConDemo.GameLauncher Editor ROSConDemo.Assets -j $CMAKE_JOBS \
        && cd $WORKSPACE/ROSConDemo/kraken_nav \
        && colcon build --symlink-install \
        && rm -rf ~/.o3de/3rdParty/ \
        && rm -rf $WORKSPACE/o3de/.git \
        && rm -rf $WORKSPACE/o3de/AutomatedTesting \
        && rm -rf $WORKSPACE/o3de/python/downloaded_packages \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/Azcg/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/CMake \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/CMakeFiles/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/External/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/Testing/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/_deps/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/cmake \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/lib/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/o3de/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/packages/;  \ 
    elif [ "${IMAGE_TYPE}" = "simulation" ]; then \
        git clone --recursive $O3DE_REPO $WORKSPACE/o3de \
        && git -C $WORKSPACE/o3de checkout $O3DE_BRANCH \
        && git -C $WORKSPACE/o3de lfs install \
        && git -C $WORKSPACE/o3de lfs pull \
        && $WORKSPACE/o3de/python/get_python.sh \
        && $WORKSPACE/o3de/scripts/o3de.sh register -ep $WORKSPACE/o3de \
        && git clone $O3DE_EXTRAS_REPO \
        && git -C $WORKSPACE/o3de-extras checkout $O3DE_EXTRAS_BRANCH \
        && $WORKSPACE/o3de/scripts/o3de.sh register -gp $WORKSPACE/o3de-extras/Gems/ROS2 \
        && git clone $ROSCON_DEMO_REPO \
        && git -C $WORKSPACE/ROSConDemo checkout $ROSCON_DEMO_BRANCH \
        && git -C $WORKSPACE/ROSConDemo lfs install \
        && git -C $WORKSPACE/ROSConDemo lfs pull \
        && $WORKSPACE/o3de/scripts/o3de.sh register -pp $WORKSPACE/ROSConDemo/Project \
        && . /opt/ros/${ROS_DISTRO}/setup.sh \
        && cd $WORKSPACE/ROSConDemo/Project \
        && cmake -B $WORKSPACE/ROSConDemo/Project/build/linux -S . -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DLY_STRIP_DEBUG_SYMBOLS=ON \
        && cmake --build $WORKSPACE/ROSConDemo/Project/build/linux --config profile --target ROSConDemo.GameLauncher ROSConDemo.Assets -j $CMAKE_JOBS \
        && rm -rf $WORKSPACE/o3de-extras/ \
        && rm -rf ~/.o3de/3rdParty/ \
        && rm -rf $WORKSPACE/o3de/.git \
        && rm -rf $WORKSPACE/o3de/AutomatedTesting \
        && rm -rf $WORKSPACE/o3de/python/downloaded_packages \
        && rm -rf $WORKSPACE/o3de/Code \
        && rm -rf $WORKSPACE/o3de/Gems \
        && rm -rf $WORKSPACE/ROSConDemo/.git \
        && rm -rf $WORKSPACE/ROSConDemo/Gem \
        && rm -rf $WORKSPACE/ROSConDemo/Source \
        && rm -rf $WORKSPACE/ROSConDemo/Levels \
        && rm -rf $WORKSPACE/ROSConDemo/ReflectionProbes \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/Azcg/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/CMake \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/CMakeFiles/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/External/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/Testing/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/_deps/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/cmake \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/lib/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/o3de/ \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/packages/ \ 
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/bin/profile/*.Editor.so \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/bin/profile/EditorPlugins \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/bin/profile/Editor \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/bin/profile/AssetProcessor \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/bin/profile/AssetProcessorBatch \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/bin/profile/MaterialEditor \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/bin/profile/AssetBuilder \
        && rm -rf $WORKSPACE/ROSConDemo/build/linux/bin/profile/MaterialCanvas; \
    elif [  "${IMAGE_TYPE}" = "navstack" ]; then \
        git clone $ROSCON_DEMO_REPO \
        && git -C $WORKSPACE/ROSConDemo checkout $ROSCON_DEMO_BRANCH \
        && git -C $WORKSPACE/ROSConDemo lfs install \
        && git -C $WORKSPACE/ROSConDemo lfs pull; \
    else \
        echo "Unsupported IMAGE_TYPE: ${IMAGE_TYPE}" && exit 1; \
    fi

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Compile the navstack and add configuration to the startup
RUN cd $WORKSPACE/ROSConDemo/kraken_nav \
    && colcon build --symlink-install \
    && echo "source /data/workspace/ROSConDemo/kraken_nav/install/setup.bash" >> ~/.bashrc

# Add the appropriate ros2 environment setup script to the system startup
RUN echo "[Unit]" > /etc/systemd/system/setup_ros.service \
    && echo "After=network.target" >> /etc/systemd/system/setup_ros.service \
    && echo "" >> /etc/systemd/system/setup_ros.service \
    && echo "[Service]" >> /etc/systemd/system/setup_ros.service \
    && if [ "${ROS_VERSION}" = "humble" ]; then \
         echo "ExecStart=/opt/ros/humble/setup.bash" >> /etc/systemd/system/setup_ros.service; \
       elif [ "${ROS_VERSION}" = "galactic" ]; then \
         echo "ExecStart=/opt/ros/galactic/setup.bash" >> /etc/systemd/system/setup_ros.service; \
       fi \
    && echo "" >> /etc/systemd/system/setup_ros.service \
    && echo "[Install]" >> /etc/systemd/system/setup_ros.service \
    && echo "WantedBy=default.target" >> /etc/systemd/system/setup_ros.service \
    && echo "" >> /etc/systemd/system/setup_ros.service


ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
