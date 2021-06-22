# UW Engineering IDEAs Clinic Autonomous Vehicle Educational Platform

testing

This project is being developed as an educational platform for undergraduate students to get exposure into the domain of autonomous vehicles.

This platform will allow students to delve into:
- Mechanical Design
- Electrical system integration
- Software and algorithm development
- IoT

This repository holds some documentation for the electrical and mechanical design but is mainly a set of ROS packages to interface with the car platform in simulation or real life.

## Prerequisites to use:

The easiest way to get up and running with the environment is Docker.

### Get Docker Running on Your Host
1. Make sure you have docker client (and daemon) installed on your host OS (see instructions at https://docs.docker.com/get-started/)
1. Check your docker daemon is running correctly. Running most basic docker client commands will verify this e.g.:
    ```bash
    docker image ls
    ```
1. If the above fails (typically on Ubuntu), you may need to restart your docker daemon:
    ```bash
    sudo service docker stop
    sudo service docker start
    ```

### Create docker image from the Dockerfile
1. Make sure you have nvidia drivers and the nvidia-cuda-toolkit installed or your container will try to use the inbuilt CPU graphics instead, and won't run properly. To install the toolkit:
    ```bash
    sudo apt-get install nvidia-cuda-toolkit
    ```
1. Install nvidia-docker by following the instructions at https://github.com/NVIDIA/nvidia-docker. This essentially boils down to:
    ```bash
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    ```

## TLDR Quick start guide to test lane following in simulation:
0. `git clone https://github.com/RVSagar/uw-auto-rc-car.git`
1. `cd uw-auto-rc-car`
3. `git submodule update --init --recursive`
4. `make` (`make vnc` if running on remote server)
6. Start the docker `./start_docker.sh latest yes` or `./start_docker_no_nvidia.sh latest yes` if you don't have an NVIDIA card.
7. `cd catkin_ws`
8. `catkin build` to build all the packages
9. `source devel/setup.bash`
10. `roslaunch auto_rc_car_demos simple_lane_demo_sim.launch` to launch Gazebo and rviz
11. Open a new terminal and type `rosrun auto_rc_car_demos simple_lane_drive.py`

`simple_lane_demo_sim.launch` has a few optional command line arguments such as `record:=true` to enable logging via a rosbag, `bag_prefix:=MY_PREFIX` to specify a file name prefix for that rosbag, and `topics:="TOPIC1 TOPIC2"` to choose what topics to record (record all if left unspecified). If multiple arguments are supplemented, separate them with whitespace.

## Experimental Tensorflow CPU Docker Image
0. `git clone https://github.com/RVSagar/uw-auto-rc-car.git`
1. `cd uw-auto-rc-car`
2. `git submodule update --init --recursive`
3. `docker pull ghcr.io/rvsagar/uw-auto-rc-car/uw_rc_car:tf-cpu` (this will take a while, it'll download the latest tf-cpu image from the GitHub Container Registry)
4. Start the docker `./start_docker_no_nvidia.sh tf-cpu yes`
5. You should now have a usable container with Tensorflow/Keras that will train on a CPU. ROS Melodic is also available in this image.

## Component Overview
- `auto_rc_car_api`: core package that contains custom messages, services and nodes to interface with a simulated or real car
- `auto_rc_car_control`: controllers for simulating the car in Gazebo
- `auto_rc_car_demos`: demo nodes and launch files for lane-following and obstacle avoidance
- `auto_rc_car_description`: URDF file and configurations for simulating car platform in Gazebo/rviz. Contains kinematic and dynamic model
- `auto_rc_car_worlds`: nodes, launch files, textures and images for creating Gazebo worlds for more realistic simulations
- `hardware_testing`: scripts and launch files to test sensors on the vehicle
- `sensor_wrappers`: lightweight wrappers that remap default sensor topics to ones used by the car API
