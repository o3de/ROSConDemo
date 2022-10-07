# ROSConDemo

ROSConDemo repo contains the ROSConDemo project for O3DE.

## **Project overview**

This project was created as a means of demonstrating the ROS2 Gem capabilities which it achieves by integrating the ROS2 libraries with the O3DE engine.

## ****Download and Install****

### Dependencies

**ROS 2 Gem**

This project uses the [ROS 2 Gem](https://github.com/RobotecAI/o3de-ros2-gem). Please make sure to follow the installation guide located in it’s [README.md](https://github.com/RobotecAI/o3de-ros2-gem/blob/development/README.md) file. To learn more about how the Gem works check out the [ROS 2 Gem user guide](https://github.com/RobotecAI/o3de-ros2-gem/blob/development/docs/guides/ros2-gem.md).

**Additional ROS 2 packages**

The Vision messages package, which can be obtained:

`sudo apt install ros-${ROS_DISTRO}-vision-msgs`

💡 ***Note:*** This is a dependency besides all the packages required by the ROS 2 Gem.

### Clone the repository 

```shell
git clone https://github.com/aws-lumberyard/ROSConDemo.git
```

For more details on the steps above, refer to [Setting up O3DE from GitHub](https://o3de.org/docs/welcome-guide/setup/setup-from-github/) in the documentation.

### Setting up new projects and building the engine

1. From the O3DE repo folder, set up a new project using the `o3de create-project` command.
    ```
    scripts\o3de.bat create-project --project-path <your new project path>
    ```
2. Configure a solution for your project.
    ```
    cmake -B <your project build path> -S <your new project source path> -G "Visual Studio 16"
    ```

    Example:
    ```
    cmake -B C:\my-project\build\windows -S C:\my-project -G "Visual Studio 16"
    ```
    
    > Note:  Do not use trailing slashes for the <3rdParty cache path>.

3. Build the project, Asset Processor, and Editor to binaries by running this command inside your project:
    ```
    cmake --build <your project build path> --target <New Project Name>.GameLauncher Editor --config profile -- /m
    ```
    
    > Note: Your project name used in the build target is the same as the directory name of your project.

This will compile after some time and binaries will be available in the project build path you've specified, under `bin/profile`.

For a complete tutorial on project configuration, see [Creating Projects Using the Command Line Interface](https://o3de.org/docs/welcome-guide/create/creating-projects-using-cli/) in the documentation.

## Level(s)

Describe level(s).

### Main Level

The main level of the demo is set in an apple orchard surrounded by a countryside. The orchard is managed by the Apple Kraken.

The Apple Kraken is a four-wheeled robot assigned the task of navigating around the orchard, collecting apples and storing them in its basket. You can choose to control the Apple Kraken manually or use the navigation stack which does that for you.

### Playground Level

### Test Level

## Apple Kraken import instructions

The AppleKraken is saved in a form of a prefab in the Project' Assets directory.

1. In your Project’s editor, you can instantiate it by clicking the right mouse button and then selecting instantiate prefab (as depicted below).
    
    ![Kraken instantiation](static/fig_1.png)
    
2. Now select the ***ROSConDemo/Project/Assets/Importer/apple_kraken.prefab*** file
    
    <img src="static/fig_2.png" width="60%" alt="AppleKraken Path">

## Triggering Apple Gathering

Check available services in a terminal using this command:
- `ros2 service list`

If your simulation is running, you should be able to see the apple gathering service listed there.
- It could be named `/trigger_apple_gathering`.

If Apple Kraken is in position, you can trigger apple gathering with a terminal command:
- `ros2 service call /trigger_apple_gathering std_srvs/srv/Trigger`

You can also cancel a gathering operation in progress by calling another service:
- `ros2 service call /cancel_apple_gathering std_srvs/srv/Trigger`

## Navigation stack

Instructions on how to run it with navigation stack and global automation.

## Troubleshooting

Troubleshooting section


## License

For terms please see the LICENSE*.TXT files at the root of this distribution.
