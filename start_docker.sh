#!/bin/bash

# Originally based on files by Alex Werner for UW Robohub
# Used with verbal permission

# Allows the user to choose what tag to run each time for flexibility
if [ -z $1 ]; then
	echo "Usage direction: ./start_docker.sh TAG_NAME LOCAL_DISPLAY [COMMAND_OVERRIDE]"
	echo "Please input the desired image tag as the first argument"
	# TODO: finish this
	echo "And a \"yes\" for using a local display or \"no\" otherwise"
	echo "Your input was: $1 $2 $3"
	exit
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"

# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_NAME=$USER
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_NAME --env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME --privileged"

# Settings required for having nvidia GPU acceleration inside the docker
if [ $2 == "yes" ]; then
	DOCKER_GPU_ARGS="--env DISPLAY=unix${DISPLAY} --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"
else
	DOCKER_GPU_ARGS="--env QT_X11_NO_MITSHM=1"
fi
# for the headless server there's no DISPLAY to display to so disabling that for now
# PS even though it's possible to start the x server on the host, the host can't use
# the GPU for graphics itself yet, plus VNC server in a container is "safer" security wise

which nvidia-docker > /dev/null 2> /dev/null
HAS_NVIDIA_DOCKER=$?
# Docker >= 19.03 supports nvidia gpus by default, nvidia-docker is deprecated
# Check if current docker version is >= 19.03
DOCKER_VERSION="$(docker version --format '{{.Client.Version}}')"

# Check if $1 version is less than or equal to $2
verlte() {
    [  "$1" = "`echo -e "$1\n$2" | sort -V | head -n1`" ]
}

if [ $HAS_NVIDIA_DOCKER -eq 0 ] || verlte 19.03 $DOCKER_VERSION; then
  # With new version of docker, maintain regular docker command with appended --gpus all tag
  if verlte 19.03 $DOCKER_VERSION; then
  	DOCKER_COMMAND=docker
	DOCKER_GPU_ARGS="--gpus all $DOCKER_GPU_ARGS"
  else
	DOCKER_COMMAND=nvidia-docker
  fi

  DOCKER_GPU_ARGS="$DOCKER_GPU_ARGS --env NVIDIA_VISIBLE_DEVICES=all --env NVIDIA_DRIVER_CAPABILITIES=all"
else
  #echo "Running without nvidia-docker, if you have an NVidia card you may need it"\
  #"to have GPU acceleration"
  DOCKER_COMMAND=docker
fi


xhost + 

#ADDITIONAL_FLAGS="--detach"
ADDITIONAL_FLAGS="--rm --interactive --tty"
ADDITIONAL_FLAGS="$ADDITIONAL_FLAGS --device /dev/dri:/dev/dri --volume=/run/udev:/run/udev"

IMAGE_NAME=uw_rc_car
TAG=$1
CONTAINER_NAME=${IMAGE_NAME}_${USER}

echo Using container: $IMAGE_NAME:$TAG

if ! docker container ps | grep -q ${CONTAINER_NAME}:${TAG}; then
	echo "Starting new container with name: ${CONTAINER_NAME}:${TAG}"
	$DOCKER_COMMAND run \
	$DOCKER_USER_ARGS \
	$DOCKER_GPU_ARGS \
	$DOCKER_SSH_AUTH_ARGS \
	-v "$DIR:/home/${USER}" \
	-v="/dev:/dev" \
	-v="/etc/udev:/etc/udev" \
	$ADDITIONAL_FLAGS --user root \
	--name ${CONTAINER_NAME} --workdir /home/$USER \
	--cap-add=SYS_PTRACE \
	--cap-add=SYS_NICE \
	--net host \
	--env USER=${USER} \
	--device /dev/bus/usb \
	$IMAGE_NAME:$TAG \
	$3
else
	echo "Starting shell in running container"
	docker exec -it --workdir /home/${USER} --user root --env USER=${USER} ${CONTAINER_NAME} bash -l -c "stty cols $(tput cols); stty rows $(tput lines); bash"
fi
