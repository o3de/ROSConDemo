# Dockerfile for running the ROSConDemo

The dockerfile defined in this path will prepare the appropiate ROS2 Humble distribution based environment and build the components necessary to run the ROSCon demo project simulator through the O3DE engine.

## Prerequisites

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* Ubuntu 20.04 (Focal) or 22.04 (Jammy)
* At least 60 GB of free disk space
* Docker installed and configured
  * **Note** It is recommended to have Docker installed correctly and in a secure manner so that the docker commands in this guide do not require elevated privileges (sudo) in order to run them. See [Docker Engine post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for more details.
* [NVidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

## Building the Docker Image

The dockerfile supports defining which version of Ubuntu+ROS to base the docker container on, and by default will support Ubuntu 22.04 (jammy) with the ROS2 Humber distribution. It will build an O3DE simulation environment that is configured to launch the O3DE editor, O3DE simulation launcher, and the simulation navigation stack used for the simulation.

To build the docker image for the ROSConDemo environment, run the following command:

```
docker build --build-arg O3DE_BRANCH=199205f --build-arg O3DE_EXTRAS_BRANCH=cbd3cd5 -t roscon_demo -f Dockerfile .
```

This will create a `roscon_demo` docker image which will used when running the container.


**Note** 
The above command example will build the full simulation environment needed to run the O3DE editor, O3DE simulation launcher, and the simulation navigation stack, based on the latest code from the o3de (O3DE Engine), o3de-extras (ROS2 Gem), and the ROSConDemo. The arguments specified will pull in the last known good version of the dependent projects from their repo. Additional arguments are available to customize and fine-tune this process and is described below.

The build process may take over two hours depending on the hardware resource and network connectivity of the machine used to build the image.

## Running the Docker Image

GPU acceleration is required for running O3DE correctly. For running docker with support for GPU please follow the documentation for [docker run](https://docs.docker.com/engine/reference/commandline/run/).

Another option is to install and use [rocker](https://github.com/osrf/rocker).

```
rocker --x11 --nvidia roscon_demo
```


To launch the O3DE editor for the ROSConDemo project, execute the following command within the docker terminal

```
/data/workspace/ROSConDemo/Project/build/linux/bin/profile/Editor
```

Further instructions and details can be found in the [main README file](https://github.com/o3de/ROSConDemo/blob/development/README.md#running-the-demo-scenario)

To launch the O3DE simulation launcher for the ROSConDemo project, execute the following command within the docker terminal

```
/data/workspace/ROSConDemo/Project/build/linux/bin/profile/./ROSConDemo.GameLauncher -LoadLevel=main
```

To spawn or launch the rviz visualizer, follow the [kraken_nav README file](https://github.com/o3de/ROSConDemo/blob/development/kraken_nav/README.md#running-simulation)

## Advanced Options
### Custom source repos and branches

The Dockerscripts use the following arguments to determine the repository to pull the source from. 

| Argument              | Repository                       | Default     |
|-----------------------|----------------------------------|-------------|
| O3DE_REPO             | O3DE                             | https://github.com/o3de/o3de.git                   |
| O3DE_EXTRAS_REPO      | O3DE Extras                      | https://github.com/o3de/o3de-extras.git            |
| ROSCON_DEMO_REPO      | ROSConDemo repository            | https://github.com/o3de/RobotVacuumSample          |


In addition to the repositories, the following arguments target the branch, commit, or tag to pull from their corresponding repository

| Argument                | Repository                       | Default     |
|-------------------------|----------------------------------|-------------|
| O3DE_BRANCH             | O3DE                             | development |
| O3DE_EXTRAS_BRANCH      | O3DE Extras                      | development |
| ROSCON_DEMO_BRANCH      | ROSConDemo repository            | main        |

### Optimizing the build process ###
The docker script provides a cmake-specific argument override to control the number of parallel jobs that can be used during the build of the docker image. `CMAKE_JOBS` sets the maximum number of concurrent jobs cmake will run during its build process and defaults to 8 jobs. This number can be adjusted to better suit the hardware which is running the docker image build.


## Creating the Simulation only Docker Images.

A slimmer docker image can be built that only contains the ROSConDemo simulation launcher and the kraken_nav navigation stack. The O3DE editor will not be part of the image, and therefore the build will take less time and less space. The argument `IMAGE_TYPE` can be set to `simulation` to create this docker image.


The Dockerfile provides arguments to build docker images that only contain the necessary files to run the O3DE simulation launcher and the navigation stack, without the need to launch the O3DE Editor. To build the the docker image for just the simulation portion of the ROSConDemo, run the following command:

```
docker build --build-arg IMAGE_TYPE=simulation --build-arg O3DE_BRANCH=199205f --build-arg O3DE_EXTRAS_BRANCH=cbd3cd5 -t roscon_demo_simulation -f Dockerfile .
```

## Creating a navigation stack only Docker Images.

A minimal docker image that only contains the built `kraken_nav` navigation stack can also be specified. This is useful to have an image separate from the simulation environment to monitor and control the simulation through the ROS2 framework. The argument `IMAGE_TYPE` will need to be set to `navstack` to build this image. The image will be even smaller since it will not contain any O3DE simulation binaries or assets. It will have the necessary ROS2 packages including the rviz visualizer to view and control the simulation. To build the navigation stack only docker image, run the following command:

```
docker build --build-arg IMAGE_TYPE=navstack --build-arg O3DE_BRANCH=199205f --build-arg O3DE_EXTRAS_BRANCH=cbd3cd5 --build-arg ROSCON_DEMO_BRANCH=development -t roscon_demo_navstack -f Dockerfile .
```



