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
    sudo apt install ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-pointcloud-to-laserscan ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-ackermann-msgs
    ```

3.  You will also need [colcon](https://colcon.readthedocs.io/en/released/user/installation.html) installed in order to build the workspace. Run the following command to install.

    ```
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

    sudo apt update
    sudo apt install python3-colcon-common-extensions
    ```

## Installation ##

- Use the [roscon_2022](https://github.com/aws-lumberyard-dev/o3de/tree/roscon_2022) branch of the `O3DE`.
- Use the the [development](https://github.com/RobotecAI/o3de-ros2-gem/tree/development) branch of the `o3de-ros-gem`.
- Use the [main](https://github.com/aws-lumberyard/ROSConDemo) branch of the `ROSConDemo`.

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
2. Load level `Main`

## Running nav stack with multiple vehicles

> Note: Slam is turned off by default since we have ground truth information about the robot's position from the simulator. However, it is possible to enable `slam_toolbox` forcefully. You can allow slam by adding `use_slam:=True` to the navigation launch command.

1. For our scenario, spawn the following robots:

```bash
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_rusty', xml: 'line1'}' &&
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_shiny', xml: 'line2'}' &&
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_rusty', xml: 'line3'}' &&
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'apple_kraken_shiny', xml: 'line4'}'
```

2. Source the workspace

```bash
cd ~/o3de_kraken_ws
source ./install/setup.bash
```

3. Set up `CycloneDDS` rmw:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

4. Run the first stack with `Rviz` param set to `True`:

```bash
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_rusty_1 rviz:=True
```

5. Run the stack for rest of the robots (in different terminals, remember about points `1-3`):

```bash
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_shiny_2 rviz:=False
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_rusty_3 rviz:=False
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=apple_kraken_shiny_4 rviz:=False
```
