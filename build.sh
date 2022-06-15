#! /bin/bash

export DOCKER_BUILDKIT=1
eval `ssh-agent`
ssh-add ~/.ssh/id_rsa
echo `ssh-add -l`
# docker build --no-cache -t brushee-camera-docker --ssh default .
docker build  -t ibukinishimura/brushee-camera-docker --ssh default --progress=plain .
# docker buildx build  -t brushee-camera-docker --ssh default --platform x86_64 .
