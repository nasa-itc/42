# Run 42 in podman within a [vnc](https://turbovnc.org/) web session

`cd ./containers`

## Dependencies:

- [podman](https://docs.podman.io/en/latest/)
- [make](https://www.gnu.org/software/make/)
- [docker](https://docs.docker.com/engine/install/) (optional)

### Install `podman`:

Follow the installation instructions [here](https://podman.io/docs/installation)

```bash
podman machine init
podman machine start
```

If you wish to learn more about how to use `podman` go [here](https://www.redhat.com/en/blog/container-information-podman)

### Build 42 inside of a `podman` image:

`make build`

### Bring up 42 inside of a `podman` container:

`make up`

After a few minutes have a browser open http://localhost:5801/vnc.html and click on 'Connect'

### Bring down `podman`'s 42 container:

`make down`

## For `docker` do the following:

#### Build 42 inside of a `docker` image:

`make build CONTAINER_BIN=docker`

#### Bring up 42 inside of a `docker` container:

`make up CONTAINER_BIN=docker`

After a few minutes have a browser open http://localhost:5801/vnc.html and click on 'Connect'

#### Bring down `docker`'s 42 container:

`make down CONTAINER_BIN=docker`

## To see all available `make` targets:

run `make` with no arguments, which returns

```bash
#---------------------------------------------------------------------------------------- 
# This Makefile can be used to run build, bring up, and bring down, in vnc  
#   open: http://localhost:5801/vnc.html  
#---------------------------------------------------------------------------------------- 

#-target-----------------------description----------------------------------------------- 
build                          Build 42 podman/docker image
clean                          Clean up: stop, and remove container and delete 42's image
delete                         Delete 42's image
down                           Bring down 42 container
info                           show info
stop                           Stop 42's container
up                             Bring up 42 via podman/docker, in x/vnc system
```
