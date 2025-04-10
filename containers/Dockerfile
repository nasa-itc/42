ARG REGISTRY_HOST=ghcr.io
ARG IMAGE_USERNAME=haisamido
ARG IMAGE_NAME=x-vnc
ARG IMAGE_TAG=latest

ARG IMAGE_URI=${REGISTRY_HOST}/${IMAGE_USERNAME}/${IMAGE_NAME}:${IMAGE_TAG}

ARG GIT_URL=https://github.com/ericstoneking/42.git
ARG GIT_BRANCH=master
ARG VNC_PASSWORD=0123456789

################################################################################
FROM ${IMAGE_URI}
################################################################################

ARG GIT_URL
ARG GIT_BRANCH
ARG VNC_PASSWORD

ENV GIT_URL=${GIT_URL}
ENV GIT_BRANCH=${GIT_BRANCH}
ENV VNC_PASSWORD=${VNC_PASSWORD}

# VNC Config
RUN mkdir -p ~/.vnc/
RUN echo ${VNC_PASSWORD} | vncpasswd -f > ~/.vnc/passwd
RUN chmod 0600 ~/.vnc/passwd
RUN openssl req -x509 -nodes -newkey rsa:2048 -keyout ~/novnc.pem -out ~/novnc.pem -days 3650 -subj "/C=US/ST=NY/L=NY/O=NY/OU=NY/CN=NY emailAddress=email@example.com"

# Install 42's dependencies
RUN apt-get update && \
    apt-get -y install libglu1-mesa-dev freeglut3-dev mesa-common-dev libglfw3-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 42's MonteCarlo dependencies
RUN apt-get update && \
    apt-get -y install octave && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# install Julia
RUN  curl -fsSL https://install.julialang.org | sh -s -- --yes

# Clone the 42 repo for a specific branch
RUN git clone --recurse-submodules -b ${GIT_BRANCH} -j2 ${GIT_URL}
RUN cd ./42 && make -j2

COPY entrypoint.sh /entrypoint.sh

COPY startapp.sh /startapp.sh

CMD ["/entrypoint.sh"]
