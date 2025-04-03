# How to use 42 in Docker/Podman within a vnc (turbovnc) web session

`cd ./docker`

## In `docker`

### To build 42 inside of a `docker` image do the following:

`make build`

### To bring up 42 inside of `docker` container do the following:

`make up`

then in a browser connect to http://localhost:5801/vnc.html and click on 'Connect'

### To bring down `docker`'s 42 do the following:

`make down`

## In `podman`

### To build 42 inside of a `podman` image do the following:

`make build CONTAINER_BIN=podman`

### To bring up 42 inside of `podman` container do the following:

`make up CONTAINER_BIN=podman`

then in a browser connect to http://localhost:5801/vnc.html and click on 'Connect'

### To bring down `podman`'s 42 do the following:

`make down CONTAINER_BIN=podman`

### Assumptions that `podman` has been run via:

Install `podman` via these instructions https://podman.io/docs/installation and then do the following:

```bash
podman machine init
podman machine start
```
