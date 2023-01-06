# Dockerfile for running the RobotHarvestingSample


The Dockerfile is going to create an image that prepares the O3DE simulator
together with the o3de-ros2-gem and o3de_kraken_nav to run the demo hosted
in this repository.

The image by default will use Ubuntu Jammy and ROS 2 Humble but can also
be configured to use Ubuntu Focal and ROS 2 Galactic.

## Requisites

 * [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
 * At least 40GB of free disk space
 * The build process can take more than 2 hours

## Building the Docker Image

To build the Dockerfile the only required step is to clone first the RobotHarvestingSample:
```
git clone git@github.com:o3de/RobotHarvestingSample.git
cd RobotHarvestingSample/docker
docker build -t robot_harvesting_sample -f Dockerfile .
```

Note: the build process is going to download all the necessary assets for running
the demo so it can take several hours depending on the Internet connection.

## Building the default Docker Image (Ubuntu Jammy + ROS 2 Humble)

Creating the image using the Dockerfile to have an Ubuntu Jammy and ROS 2
Humble platform requires just to have docker installed:

```
wget https://raw.githubusercontent.com/o3de/ROSConDemo/main/docker/Dockerfile
docker build -t roscon_demo -f Dockerfile .
```

## Building the Docker Image using Ubuntu Focal and ROS 2 Galactic

Creating the image using the Dockerfile to have an Ubuntu Focal and ROS 2
Galactic platform requires just to have docker installed and provide some
extra building arguments to it:

```
wget https://raw.githubusercontent.com/o3de/ROSConDemo/main/docker/Dockerfile
docker build -t roscon_demo_galactic --build-arg ROS_VERSION=galactic --build-arg UBUNTU_VERSION=focal  .
```


## Running the Docker Image

GPU acceleration is required for running O3DE correctly. For running docker
with support for GPU please follow the documentation for
[docker run](https://docs.docker.com/engine/reference/commandline/run/).

Another option is to install and use [rocker](https://github.com/osrf/rocker).

```
rocker --x11 --nvidia robot_harvesting_sample
```

The `Dockerfile` leaves ready the compilation of all the O3DE artifacts to be able
to execute the Editor.
```
/data/workspace/RobotHarvestingSample/Project/build/linux/bin/profile/Editor
```

Continue with the instruction in the
[main README file](../README.md).
