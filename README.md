# Apple Kraken Demo Project

This project demonstrates an example application of [O3DE](https://www.o3de.org/) working with ROS 2.
The integration is realized through [ROS 2 Gem for O3DE](https://github.com/RobotecAI/o3de-ros2-gem).

## How does it look like

## The project includes

- **Apple Orchard**, a simulation scene with many rows of apple trees.
- **Apple Kraken**, a robot tasked with apple picking. It is ready to use and also included as an URDF.
    - Multiple Apple Krakens are supported
    - .. and you can spawn them using ROS 2 messages!
- **Custom components** for picking apples, which benefit from direct integration with ROS 2.
    - Yes, you can write ROS 2 code in O3DE!
- **Autonomous operation** which is based on ROS 2 navigation stack and ground truth.
    - Ground truth can be replaced with detectors based on sensor data. Give it a try!
- **Apples**
    - Thousands of apples!

# How to build the project and its dependencies

## Requirements

### Platforms

The project supports the following platforms:

- Ubuntu 22.04 with ROS 2 Humble
- Ubuntu 20.04 with ROS 2 Galactic

### O3DE

1. Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make
   sure that the system/hardware requirements are met.
2. Please follow the instructions
   to [set up O3DE from GitHub](https://o3de.org/docs/welcome-guide/setup/setup-from-github/).
3. **Use the `roscon_2022` branch**: `git checkout roscon_2022`.

The following commands should prepare O3DE:

```
~$ git clone https://github.com/o3de/o3de.git
~$ cd o3de
~/o3de$ git lfs install
~/o3de$ git lfs pull
~/o3de$ python/get_python.sh
~/o3de$ scripts/o3de.sh register --this-engine
```

### ROS 2 Gem

This project uses the [ROS 2 Gem](https://github.com/RobotecAI/o3de-ros2-gem).
Please make sure to follow the installation guide
in [README.md](https://github.com/RobotecAI/o3de-ros2-gem/blob/development/README.md) file.
To learn more about how the Gem works check out
the [ROS 2 Gem user guide](https://github.com/RobotecAI/o3de-ros2-gem/blob/development/docs/guides/ros2-gem.md).

Note that the Gem instructions include installation of ROS 2 with some additional packages.

The Gem is open to your contributions!

#### Registering the Gem

During the step above, make sure to register the Gem in the engine:
`scripts/o3de.sh register --gem-path <PATH_TO_CLONED_ROS2_GEM>`

### Additional ROS 2 packages**

The vision messages package, which can be obtained:

`sudo apt install ros-${ROS_DISTRO}-vision-msgs`

💡 ***Note:*** This is a dependency besides all the packages already required by the ROS 2 Gem.

### Build this project

1. Clone it:

```
git clone https://github.com/aws-lumberyard/ROSConDemo.git
```

2. Register this project in O3DE engine:

in O3DE directory:
```
scripts/o3de.sh register -pp <PATH_TO_THIS_PROJECT>
```

3. Ensure your [ROS 2 is sourced](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

```
echo $ROS_DISTRO
> humble
```

4. Configure build:

```
cmake -B build/linux -G Ninja Multi-Config -DLY_STRIP_DEBUG_SYMBOLS=ON
```

5. Execute build (this will take a while the first time):

```
cmake --build build/linux --config profile --target ROSConDemo Editor AssetProcessor ROSConDemo.Assets
```

### Running the project

Launch the O3DE Editor (in the Project directory):

```
build/linux/bin/profile/Editor
```

## Levels


### Main Level

The main level of the demo is set in an apple orchard surrounded by a countryside. The orchard is managed by the Apple
Kraken.

The main level is rather performance intensive.

The Apple Kraken is a four-wheeled robot assigned the task of navigating around the orchard, collecting apples and
storing them in its basket.

### Playground Level

The playground level is much lighter and can be used to quickly prototype with Kraken. There is only a couple
of apple trees and the robot itself.

## Apple Kraken spawning instructions

TODO 

## Triggering Apple Gathering

Check available services in a terminal using this command:

- `ros2 service list`

If your simulation is running, you should be able to see the apple gathering service listed there.

- It could be named `/trigger_apple_gathering`.

If Apple Kraken is in position, next to a tree, you can trigger apple gathering with a terminal command:

- `ros2 service call /trigger_apple_gathering std_srvs/srv/Trigger`

You can also cancel a gathering operation in progress by calling another service:

- `ros2 service call /cancel_apple_gathering std_srvs/srv/Trigger`

## Navigation stack

TODO 

## Troubleshooting

#### Check-list

- Is O3DE running ok with an empty or default project?
- Is ROS 2 installation ok? (check with `ros2 topic pub` etc.)
- Is ROS 2 workspace sourced? (check `ROS_DISTRO`, `AMENT_PREFIX_PATH`)
- Do you have compatible settings for crucial ENV variables when running the navigation / orchestration stack in the
  console and when running the simulator?
    - check `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` etc.
- Check console for errors as well as logs. From the Project folder, check `user/log/Editor.log`.
- Are simulation topics up when you play the simulation?
    - `ros2 node list` should include `/o3de_ros2_node`
    - `ros2 topic list` should include `/clock`, `/tf` and `/tf_static` regardless of robot presence.
    - topic list should also include `/pc`, `/ackermann_vel` and `/ground_truth_3D_detection` if there is a robot in the
      scene and simulation is running.
        - note that with multiple robots, these topics will be namespaced.
    - `ros2 service list` should also show several simulation and robot services such as spawning and apple gathering.

#### Other

💡 ***Note:*** Take note that these **vision_msgs** are different between Humble and Galactic,
in particular detection messages which are used by ground truth detector.

## License

For terms please see the LICENSE*.TXT files at the root of this repository.
