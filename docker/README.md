# Dockerfiles for running the ROSConDemo

The Dockerfile defined in this path will prepare the appropriate ROS2 Iron distribution based environment and build the components necessary to run the ROSCon demo project simulator through the O3DE engine.

There are three Dockerfile scripts that are designed to provide environments to both run the ROSCon demo project simulation, and to run the ROSCon editor to open and view the demo level and assets for closer inspection. (Note that the editor environment is meant for demonstrative purposes only and not intended for actual editing and authoring)

### Dockerfile
This Dockerfile will build a docker container that will have the simulation launcher (`ROSConDemo.GameLauncher`), the `kraken_nav` code for the navigation stack, and the `Editor` which can be used to view the demo level and assets.

### Dockerfile.NavStack
This Dockerfile will build a docker container that will only have the `kraken_nav` code for the navigation stack.

### Dockerfile.Simulation
This Dockerfile will build a docker container that will have the simulation launcher (`ROSConDemo.GameLauncher`) and the `kraken_nav` code for the navigation stack.


## Prerequisites

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* Ubuntu 22.04 (Jammy)
* At least 60 GB of free disk space
* Docker installed and configured
  * **Note** It is recommended to have Docker installed correctly and in a secure manner so that the docker commands in this guide do not require elevated privileges (sudo) in order to run them. See [Docker Engine post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for more details.
* [NVidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)


## Building the Full Docker Image

The dockerfile supports defining which version of Ubuntu+ROS to base the docker container on, and by default will support Ubuntu 22.04 (jammy) with the ROS2 Humble distribution. The main `Dockerfile` will build a Docker image that will contain the Editor, RosConDemo launcher, and the kraken_nav navigation stack code. To build the full docker image into a container called `roscon_demo`, run the following build command from this `docker` subfolder of this project:

```
docker build -t roscon_demo -f Dockerfile .
```

**Note** 
The above command example will build the full simulation environment needed to run the O3DE editor, O3DE simulation launcher, and the simulation navigation stack, based on the latest code from the o3de (O3DE Engine), o3de-extras (ROS2 Gem), and the ROSConDemo. The arguments specified will pull in the last known good version of the dependent projects from their repo. See the Advanced Options section below for more information.

The build process may take over two hours depending on the hardware resource and network connectivity of the machine used to build the image.


## Running the Docker Image

GPU acceleration is required for running O3DE correctly. For running docker with support for GPU please follow the documentation for [docker run](https://docs.docker.com/engine/reference/commandline/run/).

Another option is to install and use [rocker](https://github.com/osrf/rocker).

```
rocker --x11 --nvidia --network="bridge" roscon_demo
```

**Note**
The above command will log you into the docker terminal that has the `kraken_nav` code built, but will not activate the environment until you run the following command after logging in:
```
source /data/workspace/kraken_nav/install/setup.bash
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


## Building the reduced Docker Images for the Simulation/Navigation
Also included are custom Dockerscripts that will only build the components necessary to launch and run the RosConDemo simulation only:

* Dockerfile.Simulation
  The Docker image created from this script will have a release-build packaged version of the simulation as well as the compiled `kraken_nav` code for the navigation stack.
* Dockerfile.NavStack
  The Docker image created from this script will only havethe compiled `kraken_nav` code for the navigation stack.

These images will be much smaller than the full `roscon_demo` docker image. 

To build the `roscon_sim` Docker image, run the following build command from this `docker` subfolder of this project:

```
docker build -t roscon_sim -f Dockerfile.Simulation .
```

To build the `roscon_nav` Docker image, run the following build command from this `docker` subfolder of this project:

```
docker build -t roscon_nav -f Dockerfile.NavStack .
```

## Running the Simulation Docker Image

Similar to the steps to launch the `roscon_demo` image above, you can launch the simulation Docker image with the following command:

```
rocker --x11 --nvidia --network="bridge" roscon_sim
```

Once logged into the simulation docker terminal, you can launch the simulation with the following commands:

```
cd /data/workspace/ROSConDemoGamePackage
./ROSConDemo.GameLauncher
```

## Running the Navigation Stack Docker Image

Once the simulation is running, you can launch either the Navigation Stack Docker image, or launch another instance of the Simulation Docker image to control the simulation through ROS.

```
rocker --x11 --nvidia --network="bridge" roscon_nav
```

**Note**
The above command will log you into the docker terminal that has the `kraken_nav` code built, but will not activate the environment until you run the following command after logging in:
```
source /data/workspace/kraken_nav/install/setup.bash
```

From this docker terminal, you will be able to run the demo scenario described in the [main README file](https://github.com/o3de/ROSConDemo/blob/development/README.md#running-the-demo-scenario)


## Advanced Options

### Target ROS2 Distribution
The Docker script defaults to building an image based on Ubuntu 22.04 (jammy) and the ROS2 Humble distribution. This can be overridden with a combination of the `ROS_VERSION` and `UBUNTU_VERSION` arguments.

| Arguments                                 | ROS2 Distro   |
|-------------------------------------------|---------------|
| ROS_VERSION=galactic UBUNTU_VERSION=focal | galactic      |
| ROS_VERSION=humble   UBUNTU_VERSION=jammy | humble        |
| ROS_VERSION=iron     UBUNTU_VERSION=jammy | iron          |

### Custom source repos and branches

The Dockerscripts use the following arguments to determine the repository to pull the source from. 

| Argument              | Repository                       | Default                                            |
|-----------------------|----------------------------------|----------------------------------------------------|
| O3DE_REPO             | O3DE                             | https://github.com/o3de/o3de.git                   |
| O3DE_EXTRAS_REPO      | O3DE Extras                      | https://github.com/o3de/o3de-extras.git            |
| ROSCON_DEMO_REPO      | ROSConDemo repository            | https://github.com/o3de/RobotVacuumSample          |


In addition to the repositories, the following arguments target the branch, commit, or tag to pull from their corresponding repository

| Argument                | Repository                       | Default                |
|-------------------------|----------------------------------|------------------------|
| O3DE_BRANCH             | O3DE                             | main                   |
| O3DE_EXTRAS_BRANCH      | O3DE Extras                      | main                   |
| ROSCON_DEMO_BRANCH      | ROSConDemo repository            | development            |
