#!/bin/bash

export DISPLAY=host.docker.internal:0 
export XDG_RUNTIME_DIR=/tmp

# nohup openbox-session &
# nohup ./42 &

xeyes &

tail -f /dev/null
