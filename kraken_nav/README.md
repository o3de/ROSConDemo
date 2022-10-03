# Apple kraken navigation #

## ROS2 package Requirements

- `nav2` navigation package (NAV2)[https://navigation.ros.org/]
- `slam_toolbox` slam toolbox

## Installation ##

It is assumed that ROS2 `galactic` is used and the workspace dir is in `~/o3de_kraken_ws`.

1. Source ROS2

```bash
source /opt/ros/galactic/setup.bash
```

2. Put this package in some workspace directory, inside `src`

```bash
mkdir -p ~/o3de_kraken_ws/src && cd ~/o3de_kraken_ws/src
# put package here
```

3. Go to workspace dir and build this package

```bash
cd ~/o3de_kraken_ws
colcon build --symlink-install
```

## Running scene

1. Make sure you have `https://github.com/RobotecAI/o3de-ros2-gem/tree/pjaroszek/ackermann_drive_model` branch in your `o3de-ros-gem/development`.
2. Run the `Main` level from `ROSConDemo` repository - branch `mp/deappletreeized_orchad_kraken`.

## Running nav stack

1. Source the workspace

```bash
cd ~/o3de_kraken_ws
source ./install/setup.bash
```

2. Run the navigation stack

```bash
ros2 launch o3de_kraken_nav navigation.launch.py
```

