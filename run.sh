#!/bin/bash

DOCKER_IMAGE='ghcr.io/screamlab/pros_base_image:latest'
DOCKER_NETWORK='host'
# Run the Docker container with the specified options
docker run -it --rm \
    -v $(pwd)/pros_gps:/workspaces/pros_gps\
    --device /dev/usb_gps:/dev/usb_gps \
    --network $DOCKER_NETWORK \
    $DOCKER_IMAGE \
    /bin/bash