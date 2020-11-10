SHELL := /bin/bash

base:
	docker build -t uw_rc_car:latest -t uw_rc_car:base -f docker/Dockerfile.Base .

tf:
	docker build -t uw_rc_car:latest -t uw_rc_car:tf -f docker/Dockerfile.Tensorflow .

vnc: base
	docker build -t uw_rc_car:latest -t uw_rc_car:vnc -f docker/Dockerfile.VNC .

# Obsolete
nvidia:
	docker build -t uw_rc_car:latest -t uw_rc_car:nvidia -f docker/DockerfileNvidia --build-arg BASE_IMAGE=uw_rc_car:base .
