SHELL := /bin/bash

base:
	docker build --network=host -t uw_rc_car:latest -t uw_rc_car:base -f docker/Dockerfile.Base .

tf:
	docker build -t uw_rc_car:latest -t uw_rc_car:tf -f docker/Dockerfile.Tensorflow .

# Obsolete
base_vnc: 
	docker build --network=host -t uw_rc_car:latest -t uw_rc_car:vnc -f docker/DockerfileBaseVNC .

# Obsolete
nvidia:
	docker build -t uw_rc_car:latest -t uw_rc_car:nvidia -f docker/DockerfileNvidia --build-arg BASE_IMAGE=uw_rc_car_base .
