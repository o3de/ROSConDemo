# ROSConDemo

ROSConDemo repo contains the ROSConDemo project for O3DE.

## Contribute
For information about contributing to Open 3D Engine, visit [https://o3de.org/docs/contributing/](https://o3de.org/docs/contributing/).

## Download and Install

### Additional ROS 2 packages

This project uses Vision messages, which can be obtained:

#### For ROS 2 Humble:
`sudo apt install ros-humble-vision-msgs`

#### For ROS 2 Galactic
`sudo apt install ros-galactic-vision-msgs`

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

## License

For terms please see the LICENSE*.TXT files at the root of this distribution.
