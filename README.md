## UW Engineering IDEAs Clinic Autonomous Vehicle Educational Platform

This project is being developed as an educational platform for undergraduate students to get exposure into the domain of autonomous vehicles.

This platform will allow students to delve into:
- Mechanical Design
- Electrical system integration
- Software and algorithm development
- IoT

This repository holds some documentation for the electrical and mechanical design but is mainly a set of ROS packages to interface with the car platform in simulation or real life.

TLDR Quick start guide to test lane following in simulation:
1. Go to root folder
2. `git submodule update --init --recursive`
3. `make`
4. `make nvidia` (if you have an NVIDIA GPU)
5. Start the docker `./start_docker.sh`
6. `cd caktin_ws`
7. `catkin build` to build all the packages
8. `source devel/setup.bash`
9. `roslaunch auto_rc_car_demos simple_lane_demo_sim.launch` to launch Gazebo and rviz
10. Open a new terminal and type `rosrun auto_rc_car_demos simple_lane_drive.py`
