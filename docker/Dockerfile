# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

ARG ROS_VERSION=jazzy
ARG UBUNTU_VERSION=noble

FROM ros:${ROS_VERSION}-ros-base-${UBUNTU_VERSION}

ARG ROS_VERSION
ARG UBUNTU_VERSION

# O3DE SDK installer URL and install path
ARG O3DE_DEB_URL=https://o3debinaries.org/main/Latest/Linux/o3de_2605_0.deb
ARG O3DE_INSTALL_DIR=/opt/O3DE/26.05

# ROSConDemo Variables
ARG ROSCON_DEMO_REPO=https://github.com/o3de/ROSConDemo.git
ARG ROSCON_DEMO_BRANCH=development
ARG ROSCON_DEMO_COMMIT=HEAD

ENV WORKSPACE=/data/workspace
ENV ROSCON_DEMO_ROOT=$WORKSPACE/ROSConDemo
ENV ROSCON_DEMO_PROJECT=$ROSCON_DEMO_ROOT/Project
ENV ROSCON_DEMO_NAV_ROOT=$ROSCON_DEMO_ROOT/kraken_nav
ENV LANG=en_US.UTF-8

WORKDIR $WORKSPACE

RUN apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends wget curl \
    && rm -rf /var/lib/apt/lists/*

# Install build tools, O3DE system dependencies, and ROS 2 navigation packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    binutils \
    clang \
    cmake \
    git \
    git-lfs \
    libglu1-mesa-dev \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-xinput-dev \
    libxcb-xfixes0-dev \
    libxcb-xkb-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    libfontconfig1-dev \
    libcurl4-openssl-dev \
    libsdl2-dev \
    zlib1g-dev \
    mesa-common-dev \
    libssl-dev \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-randr0 \
    libpcre2-16-0 \
    ninja-build \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-cyclonedds \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --break-system-packages python-statemachine

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Download and install the O3DE SDK
RUN wget -q -O /tmp/o3de_latest.deb ${O3DE_DEB_URL} \
    && apt-get install -y /tmp/o3de_latest.deb \
    && rm /tmp/o3de_latest.deb \
    && rm -rf /var/lib/apt/lists/*

# Initialize O3DE Python environment, register the engine, and download required Gems
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && ${O3DE_INSTALL_DIR}/python/get_python.sh \
    && ${O3DE_INSTALL_DIR}/scripts/o3de.sh register --this-engine \
    && ${O3DE_INSTALL_DIR}/scripts/o3de.sh register --repo-uri https://canonical.o3de.org \
    && ${O3DE_INSTALL_DIR}/scripts/o3de.sh download --gem-name LevelGeoreferencing==1.0.0 \
    && ${O3DE_INSTALL_DIR}/scripts/o3de.sh download --gem-name ROS2==4.2.0 \
    && ${O3DE_INSTALL_DIR}/scripts/o3de.sh download --gem-name ROS2Controllers==1.1.0 \
    && ${O3DE_INSTALL_DIR}/scripts/o3de.sh download --gem-name ROS2Sensors==1.0.1

# Clone the ROSConDemo
RUN git clone --single-branch -b $ROSCON_DEMO_BRANCH $ROSCON_DEMO_REPO $ROSCON_DEMO_ROOT \
    && git -C $ROSCON_DEMO_ROOT lfs install \
    && git -C $ROSCON_DEMO_ROOT lfs pull \
    && git -C $ROSCON_DEMO_ROOT reset --hard $ROSCON_DEMO_COMMIT

# Register the project and build the simulation
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && ${O3DE_INSTALL_DIR}/scripts/o3de.sh register -pp $ROSCON_DEMO_PROJECT \
    && cmake -B $ROSCON_DEMO_PROJECT/build/linux \
    -S $ROSCON_DEMO_PROJECT \
    -G "Ninja Multi-Config" \
    -DLY_DISABLE_TEST_MODULES=ON \
    -DLY_STRIP_DEBUG_SYMBOLS=TRUE \
    && cmake --build $ROSCON_DEMO_PROJECT/build/linux --config profile \
    --target ROSConDemo ROSConDemo.GameLauncher ROSConDemo.Assets \
    && rm -rf ~/.o3de/3rdParty/ \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/Azcg/ \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/CMake \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/CMakeFiles/ \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/External/ \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/Testing/ \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/_deps/ \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/cmake \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/lib/ \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/o3de/ \
    && rm -rf $ROSCON_DEMO_PROJECT/build/linux/packages/

# Build the navigation stack
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd $ROSCON_DEMO_NAV_ROOT \
    && colcon build --symlink-install
