#! /usr/bin/bash
#docker build --rm \
#    --build-arg BASE_IMAGE=ubuntu:20.04 \
#    -t erl/ubuntu-desktop:20.04 .

CUDA_VERSION="12.3.2-cudnn9-devel-ubuntu20.04"
docker build --rm \
    --build-arg BASE_IMAGE=nvidia/cuda:${CUDA_VERSION} \
    -t erl/ubuntu-desktop:${CUDA_VERSION} .
