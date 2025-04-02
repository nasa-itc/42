#!/bin/bash

/opt/TurboVNC/bin/vncserver -securitytypes tlsnone,x509none,none && \
websockify -D \
--web=/usr/share/novnc/ \
--cert=~/novnc.pem 80 localhost:5901

/startapp.sh

tail -f /dev/null
