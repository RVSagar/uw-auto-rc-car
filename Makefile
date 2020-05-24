SHELL := /bin/bash

base:
	docker build --network=host -t uw_rc_car_latest -t uw_rc_car_base -f docker/DockerfileBase .

base_vnc: 
	docker build --network=host -t uw_rc_car_vnc_latest -t uw_rc_car_vnc -f docker/DockerfileBaseVNC .

nvidia:
	docker build -t uw_rc_car_latest -t uw_rc_car_nvidia -f docker/DockerfileNvidia --build-arg BASE_IMAGE=uw_rc_car_base .
