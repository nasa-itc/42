
ARG REGISTRY_HOST=docker.io
ARG IMAGE_USERNAME=library
ARG IMAGE_NAME=ubuntu
ARG IMAGE_TAG=24.04

ARG IMAGE_URI=${REGISTRY_HOST}/${IMAGE_USERNAME}/${IMAGE_NAME}:${IMAGE_TAG}

ARG GIT_URL=https://github.com/ericstoneking/42.git
ARG GIT_BRANCH=master

################################################################################
FROM ${IMAGE_URI} as fortytwo-base
################################################################################
ARG OS_BASE
ARG OS_IMAGE_URI

#---
ARG GIT_URL
ARG GIT_BRANCH

ENV GIT_URL=${GIT_URL}
ENV GIT_BRANCH=${GIT_BRANCH}
#---

#---
RUN apt-get update 
RUN apt install -y x11vnc x11-apps xvfb novnc net-tools xdotool openbox menu git

# Source: https://github.com/gklingler/docker3d/blob/master/ubuntuWithOpenSourceDrivers/Dockerfile
#  with hardware accelerated OpenGL support.
RUN apt-get install -y mesa-utils
RUN apt-get install -y xserver-xorg-video-all
#---

RUN apt-get install -y build-essential
RUN apt-get install -y freeglut3-dev
#freeglut-devel

################################################################################
FROM fortytwo-base as fortytwo-dev
################################################################################
# make 42
WORKDIR /opt
RUN git clone --recurse-submodules -b ${GIT_BRANCH} -j2 ${GIT_URL}

WORKDIR /opt/42
RUN make

################################################################################
FROM fortytwo-dev as fortytwo
################################################################################
COPY entrypoint.sh /entrypoint.sh
