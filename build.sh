#! /bin/bash

export DOCKER_BUILDKIT=1
docker build -t brushee-camera-docker --ssh default .
