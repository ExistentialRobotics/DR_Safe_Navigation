#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd $(dirname $0); pwd)
cd $SCRIPT_DIR/ubuntu-desktop-20.04
./build.bash
cd $SCRIPT_DIR/ubuntu-desktop-dev
./build.bash
cd $SCRIPT_DIR/ros-noetic
./build.bash
cd $SCRIPT_DIR/clf-cbf-jackal
./build.bash
