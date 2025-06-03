#! /usr/bin/bash
CUDA_VERSION="12.3.2-cudnn9-devel-ubuntu20.04"
docker build --rm -t erl/ros-noetic:${CUDA_VERSION} --build-arg BASE_IMAGE=erl/ubuntu-desktop:${CUDA_VERSION} .
#docker build --rm -t erl/ros-noetic:cpu --build-arg BASE_IMAGE=erl/ubuntu-desktop:20.04 .
