#!/bin/bash
 
IMAGE_NAME="robotics2project"
IMAGE_TAG="jazzy"
CONTAINER_NAME="robotics2"
 
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
 
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container: ${CONTAINER_NAME}"
    docker rm -f "${CONTAINER_NAME}"
fi
 
echo "Starting container: ${CONTAINER_NAME}"
echo "Image     : ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Workspace : ${PROJECT_ROOT}/ros_ws/src -> /ros_ws/src"
 
docker run -it \
  --name "${CONTAINER_NAME}" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e ROS_DOMAIN_ID=0 \
  -e TURTLEBOT3_MODEL=burger \
  -v "${PROJECT_ROOT}/ros_ws/src:/ros_ws/src" \
  --privileged \
  "${IMAGE_NAME}:${IMAGE_TAG}"
 