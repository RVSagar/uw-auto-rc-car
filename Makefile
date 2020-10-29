SHELL := /bin/bash

base:
	docker build --network=host -t uw_rc_car:latest -t uw_rc_car:base -f docker/DockerfileBase .

melodic:
	docker build -t uw_rc_car:latest -t uw_rc_car:melodic -f docker/Dockerfile.Melodic .

base_vnc: 
	docker build --network=host -t uw_rc_car:latest -t uw_rc_car:vnc -f docker/DockerfileBaseVNC .

nvidia:
	docker build -t uw_rc_car:latest -t uw_rc_car:nvidia -f docker/DockerfileNvidia --build-arg BASE_IMAGE=uw_rc_car_base .
