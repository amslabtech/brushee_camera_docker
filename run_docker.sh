#!/bin/bash

if [ -n "`lsusb |grep Ricoh`" ]; then
    echo "Theta-s detected."
    BUS_NUMBER=`lsusb |grep Ricoh | awk '{print $2}' | sed 's/://g'`
    # echo $BUS_NUMBER
    DEVICE_NUMBER=`lsusb |grep Ricoh | awk '{print $4}' | sed 's/://g'`
    # echo $DEVICE_NUMBER
else
    echo "Theta-s not found. Did you powered on?"
    echo -n "Are you sure you want to continue? [y/N] "
    read ANSWER
    case $ANSWER in
        [Yy]* )
            echo "boot docker without theta-s"
            ;;
        *)
            echo "exitting..."
            exit 1
            ;;
    esac
fi

xhost +local:root

echo "=== run_docker ==="

IMAGE_NAME=brushee-camera-docker:latest

docker run -it --rm \
    --net='host' \
    --name="human-detection-docker" \
    --runtime="nvidia" \
    --env="DISPLAY" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device="/dev/bus/usb/$BUS_NUMBER/$DEVICE_NUMBER:/dev/bus/usb/$BUS_NUMBER/$DEVICE_NUMBER:mwr" \
    $IMAGE_NAME
