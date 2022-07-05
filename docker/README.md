# Docker images for Pilot.Auto

We have two types of Docker image: `development` and `prebuilt`.

1. The `development` image enables you to develop Pilot.Auto without setting up the local development environment.
2. The `prebuilt` image contains executables and enables you to try out Pilot.Auto quickly.
   - Note that the prebuilt image is not designed for deployment on a real vehicle!

**Note**: Before proceeding, confirm and agree with the [NVIDIA Deep Learning Container license](https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license). By pulling and using the Pilot.Auto images, you accept the terms and conditions of the license.

## Prerequisites

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [rocker](https://github.com/osrf/rocker)
  - We use `rocker` to enable GUI applications such as `rviz` and `rqt` on Docker Containers.
  - Refer to [here](http://wiki.ros.org/docker/Tutorials/GUI) for more details.

The [setup script](https://github.com/autowarefoundation/autoware/tree/main/setup-dev-env.sh) will install these dependencies through the following roles.

- [Docker](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/docker_engine/README.md)
- [rocker](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/rocker/README.md)

## Usage

### Development image

```bash
docker run --rm -it \
  -v {path_to_your_workspace}:/autoware \
  ghcr.io/tier4/pilot-auto:latest
```

To run with `rocker`:

```bash
rocker --nvidia --x11 --user \
 --volume {path_to_your_workspace} \
 -- ghcr.io/tier4/pilot-auto:latest
```

If you locate your workspace under your home directory, you can use the `--home` option instead:

```bash
rocker --nvidia --x11 --user --home \
  -- ghcr.io/tier4/pilot-auto:latest
```

To use a customized `.bashrc` for the container:

```bash
rocker --nvidia --x11 --user --home \
  --volume $HOME/.bashrc.container:$HOME/.bashrc \
  -- ghcr.io/tier4/pilot-auto:latest
```

### Prebuilt image

```bash
docker run --rm -it \
  ghcr.io/tier4/pilot-auto:latest-prebuilt
```

To run with `rocker`:

```bash
rocker --nvidia --x11 --user \
  --volume {path_to_your_workspace} \
  -- ghcr.io/tier4/pilot-auto:latest-prebuilt
```

If you intend to use pre-existing data such as maps or Rosbags, modify the `--volume` options shown below.

```bash
rocker --nvidia --x11 --user \
  --volume {path_to_your_workspace} \
  --volume {path_to_your_map_data} \
  --volume {path_to_your_log_data} \
  -- ghcr.io/tier4/pilot-auto:latest-prebuilt
```

## Building Docker images on your local machine

If you want to build these images locally for development purposes, run the following command:

```bash
cd autoware/
./docker/build.sh
```

To build without CUDA, use the `--no-cuda` option:

```bash
./docker/build.sh --no-cuda
```

To specify the platform, use the `--platform` option:

```bash
./docker/build.sh --platform linux/amd64
./docker/build.sh --platform linux/arm64
```

## Tips

### Precautions for not using `rocker`

If either image is run without `rocker`, then `root` privileges will be used.
This can affect your local environment as below:

```sh-session
$ docker run --rm -it -v {path_to_your_workspace}:/autoware ghcr.io/tier4/pilot-auto:latest
# colcon build
# exit
$ rm build/COLCON_IGNORE
rm: remove write-protected regular empty file 'build/COLCON_IGNORE'? y
rm: cannot remove 'build/COLCON_IGNORE': Permission denied
```

To prevent this error occurring when rocker is not used, there are two suggested methods:

1. Prepare a dedicated workspace for the docker image.
2. Use Visual Studio Code's [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

   To use the extension, the following settings can be used to create a user account in a similar way to `rocker.  
   Refer to [this document](https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user) for more details.

   ```jsonc
   // .devcontainer/devcontainer.json
   {
     "name": "Pilot.Auto",
     "build": {
       "dockerfile": "Dockerfile"
     },
     "remoteUser": "autoware",
     "settings": {
       "terminal.integrated.defaultProfile.linux": "bash"
     }
   }
   ```

   ```docker
   # .devcontainer/Dockerfile
   FROM ghcr.io/tier4/pilot-auto:latest

   ARG USERNAME=autoware
   ARG USER_UID=1000
   ARG USER_GID=$USER_UID

   RUN groupadd --gid $USER_GID $USERNAME \
     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
     && apt-get update \
     && apt-get install -y sudo \
     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
     && chmod 0440 /etc/sudoers.d/$USERNAME
   ```

### Using Docker images other than `latest`

There are also images versioned based on the `date` or `release tag`.  
Use them when you need a fixed version of the image.

The list of versions can be found [here](https://github.com/tier4/autoware/packages).
