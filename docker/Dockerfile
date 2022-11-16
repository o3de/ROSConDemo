ARG ROS_VERSION=humble
ARG UBUNTU_VERSION=jammy
FROM ros:${ROS_VERSION}-ros-core-${UBUNTU_VERSION}

ARG ROS_VERSION
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
    clang-12 \
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
    ros-${ROS_VERSION}-ackermann-msgs \
    ros-${ROS_VERSION}-control-toolbox \
    ros-${ROS_VERSION}-gazebo-msgs \
    ros-${ROS_VERSION}-joy \
    ros-${ROS_VERSION}-navigation2 \
    ros-${ROS_VERSION}-rviz2 \
    ros-${ROS_VERSION}-tf2-ros \
    ros-${ROS_VERSION}-urdfdom \
    ros-${ROS_VERSION}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Focal needs a modern version for CMake since it
RUN if [ "${ROS_VERSION}" = "galactic" ]; then \
      wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
        gpg --dearmor - | \
        tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
      && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | \
        tee /etc/apt/sources.list.d/kitware.list >/dev/null \
      && apt-get update && apt-get install -y cmake=3.24.1-0kitware1ubuntu20.04.1 \
      && rm -rf /var/lib/apt/lists/*; \
    fi

## Symlink clang version to non-versioned clang and set cc to clang
RUN update-alternatives --install /usr/bin/cc cc /usr/bin/clang-12 100 \
    && update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++-12 100

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
    && . /opt/ros/${ROS_VERSION}/setup.sh \
    && cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_STRIP_DEBUG_SYMBOLS=ON

RUN . /opt/ros/${ROS_VERSION}/setup.sh \
    && cmake --build build/linux --config profile --target ROSConDemo Editor AssetProcessor

# This final step takes long since they Assets will be downloading
RUN . /opt/ros/${ROS_VERSION}/setup.sh \
    && echo "This final step can take more than 1 hour. Good time for going for a coffee :)" \
    && cmake --build build/linux --config profile --target ROSConDemo.Assets

# Installing o3de_kraken_nav
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-${ROS_VERSION}-cyclonedds \
    ros-${ROS_VERSION}-rmw-cyclonedds-cpp \
    ros-${ROS_VERSION}-slam-toolbox \
    ros-${ROS_VERSION}-navigation2 \
    ros-${ROS_VERSION}-nav2-bringup \
    ros-${ROS_VERSION}-pointcloud-to-laserscan \
    ros-${ROS_VERSION}-teleop-twist-keyboard \
    ros-${ROS_VERSION}-ackermann-msgs \
    ros-${ROS_VERSION}-topic-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip install python-statemachine

WORKDIR $WORKDIR

RUN mkdir -p o3de_kraken_ws/src \
    && cd o3de_kraken_ws/src \
    && git clone https://github.com/RobotecAI/o3de_kraken_nav.git

RUN cd o3de_kraken_ws \
    && colcon build --symlink-install

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
