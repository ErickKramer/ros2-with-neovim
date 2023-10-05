#!/usr/bin/env bash

# Default values
NEOVIM_VERSION=v0.9.2
NEOVIM_PATH=~/open_source/neovim/

while [[ "${1}" ]]; do
  case "${1}" in
  -v|--version)
    shift
    if [[ "${1}" ]]; then
      NEOVIM_VERSION="${1}"
    else
      echo "Error: Version argument missing after -v or --version."
      exit 1
    fi
    ;;
  -p|--path)
    shift
    if [[ "${1}" ]]; then
      NEOVIM_PATH="${1}"
    else
      echo "Error: Path argument missing after -p or --path."
      exit 1
    fi
    ;;
  --)
    shift
    break
    ;;
  *)
    echo "Unknown option: ${1}"
    exit 1
    ;;
  esac
  shift
done

echo "========================================"
echo "Installing Neovim version ${NEOVIM_VERSION} to ${NEOVIM_PATH}"
read -rp "Press Enter to continue..."
echo "========================================"

echo "Installing pre-requisites"
sudo apt-get update
#
sudo apt-get install -y clang \
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
    unzip

pip3 install black debugpy ruff

echo "========================================"
#
echo "Installing neovim"
REPOSITORY_URL="https://github.com/neovim/neovim"
if ! (git ls-remote --heads "${REPOSITORY_URL}" "${NEOVIM_VERSION}" | grep -q "${NEOVIM_VERSION}" || \
      git ls-remote --tags "${REPOSITORY_URL}" "${NEOVIM_VERSION}" | grep -q "refs/tags/${NEOVIM_VERSION}"); then
    echo "Branch or tag '${NEOVIM_VERSION}' does not exist in the remote repository."
fi
git clone "${REPOSITORY_URL}" "${NEOVIM_PATH}" -b "${NEOVIM_VERSION}"

(
  cd "${NEOVIM_PATH}" || exit 1
  make CMAKE_BUILD_TYE=RelWithDebInfo
  sudo make install
)

echo "========================================"
echo "Configuring Neovim"
cp -r .config ~/.config

nvim --headless "+Lazy! sync" +qa

echo "\n========================================"
echo "All done!"
echo "Have fun developing with neovim!"
echo "========================================"
