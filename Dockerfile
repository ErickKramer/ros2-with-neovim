FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=rr-user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG NEOVIM_VERSION=v0.9.2

RUN apt-get update \
  && apt-get install -y \
    clang \
    clang-tidy \
    clang-format \
    cmake \
    curl \
    gdb \
    git \
    gettext \
    ninja-build \
    nodejs \
    npm \
    python3-pip \
    python3-venv \
    ripgrep \
    unzip \
    wget

RUN pip3 install \
  black \
  debugpy \
  ruff

# Setup User
RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
  && apt-get install -y sudo \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME}\
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}

WORKDIR /home/${USERNAME}

# Install neovim
COPY . /home/${USERNAME}/ros-2-with-neovim
RUN cd /home/${USERNAME}/ros-2-with-neovim \
  && ./deploy.sh -v ${NEOVIM_VERSION}

RUN mkdir -p /home/${USERNAME}/ws/src

WORKDIR /home/${USERNAME}/ws

# Clean up apt cache
RUN sudo apt-get autoremove -y \
   && sudo apt-get clean -y \
   && sudo rm -rf /var/lib/apt/lists/*
