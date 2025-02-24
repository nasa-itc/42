.DEFAULT_GOAL := help

.PHONY: x11 vnc build x11-brew x11-setup

SHELL=bash
export DIFF_PROGRAM:=vimdiff
# Containerization Parameters
export CONTAINER_BIN          :=docker
export CONTAINER_COMPOSE      :=docker compose
export DEFAULT_IMAGE_REGISTRY :=docker.io/library/
export DOCKERFILE             :=./Dockerfile

GIT_URL=https://github.com/ericstoneking/42.git
GIT_BRANCH=master

RESET  = \033[0m
PURPLE = \033[0;35m
GREEN  = \033[0;32m
LINE   = $(PURPLE)----------------------------------------------------------------------------------------$(RESET)

# Install brew if you don't have it
brew:
	@command -v brew || /usr/bin/ruby -e "$$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

# Install X11/Xquartz if you don't have it
x11-brew: brew
	brew list --cask xquartz || brew install --cask xquartz
	open /Applications/Utilities/XQuartz.app

x11-setup: x11-brew
	defaults write org.macosforge.xquartz.X11 depth -int 32 && \
	defaults write org.macosforge.xquartz.X11.plist nolisten_tcp 0 && \
	defaults write org.macosforge.xquartz.X11.plist no_auth 1

CONTAINER_NAME=42
TAG=42

42_NETWORK=nos3_core

_NUM_CPUS=$(shell nproc)
NUM_CPUS=$(shell expr $(_NUM_CPUS) / 2 )

42-create-network:
	${CONTAINER_BIN} network create \
		--driver=bridge \
		--subnet=192.168.41.0/24 \
		--gateway=192.168.41.1 \
		${42_NETWORK} || true

42-build: ## Build 42, but add x11vnc xvfb, and then tag
	${CONTAINER_BIN} build --file ${DOCKERFILE} \
		--build-arg GIT_URL=${GIT_URL} \
		--build-arg GIT_BRANCH=${GIT_BRANCH} \
		-t ${TAG} .

# docker network create nos3_sc_1
DOCKER_GATEWAY_HOST=172.17.0.1
42-up: | 42-down 42-build x11-setup 42-create-network ## bring up nos3 system, without yamcs-clean
	$(call print_message,33,Bringing up ${IMAGE} from ${TAG}...)
	xhost + && \
	${CONTAINER_BIN} run -d --rm \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-e DISPLAY=host.docker.internal:0 \
		-e XDG_RUNTIME_DIR=/tmp \
		--volume ${PWD}/entrypoint.sh:/entrypoint.sh \
		--name ${CONTAINER_NAME} \
		--network nos3_sc_1 \
		-h fortytwo \
		-p 4278:4278 \
		-p 4245:4245 \
		${TAG} &

#DOCKER_GATEWAY_HOST=172.17.0.1

# run-gsw: x11-setup 42-create-network
# 	xhost +Local:* && xhost + && \
# 	${CONTAINER_BIN} run -d \
# 	    --rm \
# 		-v /tmp/nos3:/tmp/nos3  \
# 		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
# 		-e DISPLAY=host.docker.internal:0 \
# 		-e QT_X11_NO_MITSHM=1 \
# 		--name cosmos_openc3-operator_1 \
# 		--network=${42_NETWORK} \
# 		ballaerospace/cosmos:4.5.0

42-down: ## Bring down 42
	docker stop ${TAG} || true

#	$(CONTAINER_COMPOSE) down -v --remove-orphans

x11: build x11-setup ## Run 42 in host's x windows
	xhost + && \
	$(CONTAINER_COMPOSE) up 42-x11

browser:
	open http://localhost:16920/vnc.html
	
vnc: build ## Run 42 in host's vnc client / web browser
	$(CONTAINER_COMPOSE) up 42-vnc

# x11-brew:
# 	brew search xquartz || brew install xquartz
# 	open /Applications/Utilities/XQuartz.app

# x11-setup: x11-brew
# 	defaults write org.macosforge.xquartz.X11 depth -int 32 && \
# 	defaults write org.macosforge.xquartz.X11.plist nolisten_tcp 0 && \
# 	defaults write org.macosforge.xquartz.X11.plist no_auth 1

# 42-up: pull-gsw x11-setup nos3-create-network
# 	xhost + && \
# 	${CONTAINER_BIN} run -d \
# 	    --rm \
# 		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
# 		-e DISPLAY=host.docker.internal:0 \
# 		-e QT_X11_NO_MITSHM=1 \
# 		--name ${TAG} \
# 		--network=${42_NETWORK} \
# 		ballaerospace/cosmos:4.5.0

help:
	@echo
	@echo "#$(LINE)"
	@printf "\033[37m%-30s\033[0m %s\n" "# Makefile Help                                                                         |"
	@echo "#$(LINE)"
	@printf "\033[37m%-30s\033[0m %s\n" "# This Makefile can be used to run build, and run pgmodeler in hosts's x-windows or vnc |"
	@echo "#$(LINE)"
	@echo 
	@printf "\033[37m%-30s\033[0m %s\n" "#-target-----------------------description-----------------------------------------------"
	@grep -E '^[a-zA-Z_-].+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'
	@echo 

print-%  : ; @echo $* = $($*)

