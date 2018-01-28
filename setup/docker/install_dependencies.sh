#!/bin/bash

set -euxo pipefail

apt-get update
apt install --no-install-recommends \
  terminator \
  tmux \
  nano \
  gedit \
  git \
  openssh-client \
  unzip \
  htop \
  libopenni-dev \
  apt-utils \
  usbutils \
  dialog \
  python-pip \

# these following three are ElasticFusion dependencies
apt install --no-install-recommends \
  libglew-dev \
  libsuitesparse-dev \
  libeigen3-dev

pip install -U pip setuptools

# Dependencies of trimesh
pip install numpy scipy pyassimp pyglet

# Tools for process plyfile
pip install plyfile

# python image processing
pip install scikit-image
pip install imutils


