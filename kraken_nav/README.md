# Apple kraken stack #

This repository contains two ROS2 packages:
 - o3de_kraken_nav
 - o3de_kraken_orchestration

## Packages Description

### O3DE Kraken Navigation

Provides navigation capabilities for the Apple Kraken vehicle.

### O3DE Kraken Orchestration

Allows the Apple Kraken vehicle to operate autonomously.

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

You will need to install the appropriate ROS2 package. Refer to setup requirements for the [ROS2 Gem](https://github.com/RobotecAI/o3de-ros2-gem/blob/development/README.md)

In addition to the required packages for the ROS2 gem, you will also need some additional ROS2 packages.

1.  Make sure to source the proper distribution's setup script

    For Ubuntu 20.04 + ROS2 Galactic:
    ```
    source /opt/ros/galactic/setup.bash
    ```

    For Ubuntu 22.04 + ROS2 Humble:
    ```
    source /opt/ros/humble/setup.bash
    ```

2.  Run the following command to install the remaining required packages
    ```
    sudo apt install ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-pointcloud-to-laserscan ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-topic-tools
    ```

3.  You will also need [colcon](https://colcon.readthedocs.io/en/released/user/installation.html) installed in order to build the workspace. Run the following command to install.

    ```
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

    sudo apt update
    sudo apt install python3-colcon-common-extensions
    ```

### Libraries

The o3de_kraken_orchestration package uses the [python-statemachine](https://pypi.org/project/python-statemachine/) library.\
You can install it by running the following command
```bash
pip install python-statemachine
```

**Note:** Make sure you have the pip Python package manager installed.

## Installation ##

- Use the [development](https://github.com/o3de/o3de/tree/development) branch of the `O3DE`.
- Use the the [development](https://github.com/RobotecAI/o3de-ros2-gem/tree/development) branch of the `o3de-ros-gem`.
- Use the [main](https://github.com/aws-lumberyard/ROSConDemo) branch of the `ROSConDemo`.

1. Source ROS2 (assumed `humble`)

```bash
source /opt/ros/humble/setup.bash
```

2. Build the workspace

```bash
cd ./kraken_nav
colcon build --symlink-install
```

## Running simulation

1. [Build](https://github.com/o3de/ROSConDemo#download-and-install) and run the `ROSConDemo`
2. Load level `Main`
3. Start a simulation by hitting `CTRL+G`

## Usage scenario

## Base step (for every scenario)
Please make sure that the enviroment is correct.
Note that you need to set it up for every console.
You can consider adding it to your `~/.bashrc`.

1. Make sure that you have ROS2 workspace sourced

```bash
source ./kraken_nav/install/setup.bash
```

2. Make sure that you have `CycleDDS` as middleware 
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Single robot teleoperation with joystick

1. Start teleoperation launch, make sure that you have your joystick plugged in
```bash
ros2 launch o3de_kraken_nav teleop.launch.py namespace:=apple_kraken_rusty_1
```

2. Spawn a single robot
```
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_rusty', xml: 'line1'}'
```

3. You should be able to drive the robot

### Single robot navigation

> Note: Slam is turned off by default since we have ground truth information about the robot's position from the simulator. However, it is possible to enable `slam_toolbox` forcefully. You can allow slam by adding `use_slam:=True` to the navigation launch command.


1. Run the navigation stack

```bash
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_rusty_1 rviz:=True
```

2. Spawn the robot
```bash
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_rusty', xml: 'line1'}'
```

3. You should be able to send goal to the robot. 
Make sure that that you use only the first `2D Goal Pose`


### Multiple robots navigation

> Note: Slam is turned off by default since we have ground truth information about the robot's position from the simulator. However, it is possible to enable `slam_toolbox` forcefully. You can allow slam by adding `use_slam:=True` to the navigation launch command.

1. Run the first navigation stack with `Rviz` param set to `True`:

```bash
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_rusty_1 rviz:=True
```

2. Run the navigation stack for rest of the robots (in separate terminals):

```bash
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_shiny_2 rviz:=False
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_rusty_3 rviz:=False
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_shiny_4 rviz:=False
```

3. Spawn multiple robots
```bash
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_rusty', xml: 'line1'}' &&
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_shiny', xml: 'line2'}' &&
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_rusty', xml: 'line3'}' &&
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_shiny', xml: 'line4'}'
```

4. You should be able to send goals to multiple robots.

### Multiple robots with Kraken stack 

In this scenario on top of four navigation stacks, the orchestrator nodes are executed.
Those are providing navigation stacks with goals and trigger apple gathering.

1. Run the first navigation stack with `Rviz` param set to `True`:

```bash
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_rusty_1 rviz:=True
```

2. Run the navigation stack for the rest of the robots (in separate terminals):

```bash
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_shiny_2 rviz:=False
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_rusty_3 rviz:=False
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_shiny_4 rviz:=False
```

3. Run the orchestration nodes for all the robots (in separate terminals):

```bash
ros2 run o3de_kraken_orchestration kraken_orchestration_node --ros-args -p robot_name:=apple_kraken_rusty_1 -p spawn_line:=line1
ros2 run o3de_kraken_orchestration kraken_orchestration_node --ros-args -p robot_name:=apple_kraken_shiny_2 -p spawn_line:=line2
ros2 run o3de_kraken_orchestration kraken_orchestration_node --ros-args -p robot_name:=apple_kraken_rusty_3 -p spawn_line:=line3
ros2 run o3de_kraken_orchestration kraken_orchestration_node --ros-args -p robot_name:=apple_kraken_shiny_4 -p spawn_line:=line4
```