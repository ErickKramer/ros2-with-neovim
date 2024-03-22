FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=rr-user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG NEOVIM_VERSION=v0.9.5

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
  pynvim \
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

# Install lazygit
RUN LAZYGIT_VERSION=$(curl -s "https://api.github.com/repos/jesseduffield/lazygit/releases/latest" | grep -Po '"tag_name": "v\K[^"]*') \
  && curl -Lo lazygit.tar.gz "https://github.com/jesseduffield/lazygit/releases/latest/download/lazygit_${LAZYGIT_VERSION}_Linux_x86_64.tar.gz" \
  && tar xf lazygit.tar.gz lazygit \
  && sudo install lazygit /usr/local/bin

WORKDIR /home/${USERNAME}/ws

# Clean up apt cache
RUN sudo apt-get autoremove -y \
  && sudo apt-get clean -y \
  && sudo rm -rf /var/lib/apt/lists/*
