# Dockerfile for running the ROSConDemo

The Dockerfile is going to create an Ubuntu Jammy + ROS Humble platform
that prepares the O3DE simulator together with the o3de-ros2-gem to run
the demo in this repository.

## Requisites

 * X Gb of free disk space

## Building the Docker Image

To build the Dockerfile the only required step is to clone first the ROSConDemo:
```
git clone git@github.com:aws-lumberyard/ROSConDemo.git
docker build -t roscon_demo -f ROSConDemo/docker/Dockerfile .
```
## Running the Docker Image

GPU acceleration is required for running O3DE correctly. For running docker
with support for GPU please follow the documentation for
[docker run](https://docs.docker.com/engine/reference/commandline/run/).

Another option is to install and use
[rocker](https://github.com/osrf/rocker).
