# Apple kraken navigation #

This package provides navigation capabilities for the apple Kraken vehicle.

## ROS2 prerequisites

### Middleware

- CycloneDDS

```
sudo apt install ros-${ROS_DISTRO}-cyclonedds ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```

It is required that `CycloneDDS` implementation is chosen in every working terminal:

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Packages

```
sudo apt install ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-pointcloud-to-laserscan ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-ackermann-msgs
```

## Installation ##

- Use the [roscon_2022](https://github.com/aws-lumberyard-dev/o3de/tree/roscon_2022) branch of the `O3DE`.
- Use the the [development](https://github.com/RobotecAI/o3de-ros2-gem/tree/development) branch of the `o3de-ros-gem`.
- Use [mp/deappletreeized_orchad_kraken](https://github.com/aws-lumberyard/ROSConDemo/tree/mp/deappletreeized_orchad_kraken) branch of the `ROSConDemo`.

1. Source ROS2 (assumed `humble`)

```bash
source /opt/ros/humble/setup.bash
```

2. Clone `o3de_kraken_nav` package to `src` directory inside a workspace directory (assumed `~/o3de_kraken_ws`), 

```bash
mkdir -p ~/o3de_kraken_ws/src && cd ~/o3de_kraken_ws/src
git clone https://github.com/RobotecAI/o3de_kraken_nav.git
```

3. Build the workspace

```bash
cd ~/o3de_kraken_ws
colcon build --symlink-install
```

## Running simulation

1. [Build](https://github.com/aws-lumberyard/ROSConDemo#download-and-install) and run the `ROSConDemo`
1. Load level `Main`

## Running nav stack

1. Source the workspace

```bash
cd ~/o3de_kraken_ws
source ./install/setup.bash
```

2. Set up `CycloneDDS` rmw

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

3. Run the navigation stack

```bash
ros2 launch o3de_kraken_nav navigation.launch.py
```

