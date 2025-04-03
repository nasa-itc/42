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
