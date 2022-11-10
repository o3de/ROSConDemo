# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

FROM ros:humble-ros-core-jammy

ENV WORKDIR=/data/workspace
WORKDIR $WORKDIR

ARG DEBIAN_FRONTEND=noninteractive

# Setup time zone and locale data (necessary for SSL and HTTPS packages)
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get -y \
    install \
    tzdata \
    locales \
    keyboard-configuration \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && update-locale LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Install common tools
# deps in https://github.com/o3de/o3de/blob/development/scripts/build/build_node/Platform/Linux/package-list.ubuntu-jammy.txt
RUN apt-get update && apt-get install -y \
    bc \
    bind9-utils \
    binutils \
    ca-certificates \
    clang \
    cmake \
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

# Gem + ROSConDemo ROS pacakges
RUN apt-get update && apt-get install -y \
    ros-humble-ackermann-msgs \
    ros-humble-control-toolbox \
    ros-humble-gazebo-msgs \
    ros-humble-joy \
    ros-humble-navigation2 \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-urdfdom \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

## Symlink clang version to non-versioned clang and set cc to clang
RUN update-alternatives --install /usr/bin/cc cc /usr/bin/clang 100 \
    && update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100

# Assumes a local checkout of private ROSConDemo
RUN git clone https://github.com/o3de/ROSConDemo.git \
    && cd ROSConDemo \
    && git lfs install \
    && git lfs pull

# Install o3de
RUN git clone https://github.com/o3de/o3de.git -b development \
    && cd o3de \
    && git lfs install \
    && git lfs pull \
    && python/get_python.sh

RUN git clone https://github.com/RobotecAI/o3de-ros2-gem.git

RUN ./o3de/scripts/o3de.sh register --this-engine \
    && ./o3de/scripts/o3de.sh register --gem-path ./o3de-ros2-gem \
    && ./o3de/scripts/o3de.sh register -pp ./ROSConDemo/Project \
    && ./o3de/scripts/o3de.sh enable-gem -gn ROS2 -pp ./ROSConDemo/Project \
    && cat /root/.o3de/o3de_manifest.json

WORKDIR $WORKDIR/ROSConDemo/Project

RUN git lfs pull \
    && . /opt/ros/humble/setup.sh \
    && cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_STRIP_DEBUG_SYMBOLS=ON

RUN . /opt/ros/humble/setup.sh \
    && cmake --build build/linux --config profile --target ROSConDemo Editor AssetProcessor

# This final step takes long since they Assets will be downloading
RUN . /opt/ros/humble/setup.sh \
    && echo "This final step can take more than 1 hour. Good time for going for a coffee :)" \
    && cmake --build build/linux --config profile --target ROSConDemo.Assets

# Installing o3de_kraken_nav
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-teleop-twist-keyboard \
    ros-humble-ackermann-msgs \
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip install python-statemachine

WORKDIR $WORKDIR

RUN mkdir -p o3de_kraken_ws/src \
    && cd o3de_kraken_ws/src \
    && git clone https://github.com/RobotecAI/o3de_kraken_nav.git

RUN cd o3de_kraken_ws \
    && colcon build --symlink-install

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
