# Dockerfile for running the ROSConDemo

The Dockerfile is going to create an Ubuntu Jammy + ROS Humble platform
that prepares the O3DE simulator together with the o3de-ros2-gem to run
the demo in this repository.

## Requisites

 * [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
 * At least 40GB of free disk space
 * The build process can take more than 2 hours

## Building the Docker Image

To build the Dockerfile the only required step is to clone first the ROSConDemo:
```
git clone git@github.com:aws-lumberyard/ROSConDemo.git
docker build -t roscon_demo -f ROSConDemo/docker/Dockerfile .
```

Note: the build process is going to download all the necessary assets for running
the demo so it can take several hours depending on the Internet connection.

## Running the Docker Image

GPU acceleration is required for running O3DE correctly. For running docker
with support for GPU please follow the documentation for
[docker run](https://docs.docker.com/engine/reference/commandline/run/).

Another option is to install and use [rocker](https://github.com/osrf/rocker).

```
rocker --x11 --nvidia roscon_demo
```

The `Dockerfile` leaves ready the compilation of all the O3DE artifacts to be able
to execute the Editor.
```
/data/workspace/ROSConDemo/Project/build/linux/bin/profile/Editor
```

Continue with the instruction in the
[main README file](https://github.com/aws-lumberyard/ROSConDemo/blob/main/README.md).
