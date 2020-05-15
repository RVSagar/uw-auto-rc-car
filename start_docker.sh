#!/bin/bash

# Originally based on files by Alex Werner for UW Robohub
# Used with verbal permission

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"




# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_NAME=$USER
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_NAME --env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME --privileged"



# Settings required for having nvidia GPU acceleration inside the docker
DOCKER_GPU_ARGS="--env DISPLAY=unix${DISPLAY} --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"

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


IMAGE_NAME=uw_rc_car_latest
CONTAINER_NAME=${IMAGE_NAME}_${USER}

echo Using container: $IMAGE_NAME

if ! docker container ps | grep -q ${CONTAINER_NAME}; then
	echo "Starting new container with name: ${CONTAINER_NAME}"
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
	$IMAGE_NAME bash
else
	echo "Starting shell in running container"
	docker exec -it --workdir /home/${USER} --user $(whoami) --env USER=${USER} ${CONTAINER_NAME} bash -l -c "stty cols $(tput cols); stty rows $(tput lines); bash"
fi

