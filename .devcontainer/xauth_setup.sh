#! /bin/bash

# Script to create a Xauthority file for the docker container
XAUTH=$HOME/.Xauthority
touch $XAUTH
export XAUTH_LOC=$XAUTH
if [ -z "$DISPLAY" ] || [ "${DISPLAY:0:1}" != ":" ]; then
    echo "DISPLAY variable is not set. Not running xhost."
    exit 0
fi
xhost +local:$USER

