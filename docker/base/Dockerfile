FROM ubuntu:20.04
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

## Install APT packages
# hadolint ignore=DL3008
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ansible \
  curl \
  git \
  gnupg \
  lsb-release \
  python3-pip \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

## Install vcstool
# hadolint ignore=DL3008
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-vcstool \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

## Setup build environment in a temporary directory
# hadolint ignore=DL3003
RUN --mount=type=ssh \
  mkdir -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts \
  && git clone git@github.com:tier4/autoware.proj.git -b main /tmp/autoware.proj \
  && cd /tmp/autoware.proj \
  && mkdir src \
  && vcs import src < autoware.proj.repos \
  && ./setup_ubuntu20.04.sh -c \
  && rm -rf /tmp/autoware.proj \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY docker/base/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/bin/bash"]
