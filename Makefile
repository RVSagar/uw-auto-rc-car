SHELL := /bin/bash

base:
	docker build -t uw_rc_car_latest -t uw_rc_car_base -f DockerfileBase .

nvidia:
	docker build -t uw_rc_car_latest -t uw_rc_car_nvidia -f DockerfileNvidia --build-arg BASE_IMAGE=uw_rc_car_base .