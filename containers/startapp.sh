#!/bin/bash

export DISPLAY=:1

cd ./42 && \
  xterm -e ./42 &

tail -f /dev/null
