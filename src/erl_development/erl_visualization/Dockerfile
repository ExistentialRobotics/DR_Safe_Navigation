FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04 as glvnd

FROM ros:melodic-ros-base

####### Make OpenGL work with nvidia ### 
# Copied from : https://github.com/osrf/rocker/blob/master/src/rocker/templates/nvidia_snippet.Dockerfile.em

## Works only for 18.04
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=glvnd /usr/share/glvnd/egl_vendor.d/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json


ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
####### Make OpenGL work with nvidia ### 


# make sure sudo is installed to be able to give user sudo access in docker
# Level 1 packages
RUN apt-get update \
 && apt-get install -y \
    ssh \
    sudo \
    cmake \
    build-essential \
    libproj-dev \
    python3-pip \
    python3-tk \
    pkgconf \
    x11-apps \
    mesa-utils \
    git

# Start your docker customizations here
# Level 2 packages
RUN apt-get update \
 && apt-get install -y \
 ros-melodic-rviz \
 ros-melodic-catkin \
 ros-melodic-tf2-geometry-msgs \
 libopencv-dev \
 libgazebo9-dev \
 libeigen3-dev \
 libboost-all-dev \
 libyaml-cpp-dev \
 gazebo9

# After all apt-get packages
RUN apt-get -y clean

# Prepare for python installs
RUN pip3 install --upgrade pip
RUN pip3 install pytest

COPY docker-files/ssh /root/.ssh/
RUN chmod 0400 /root/.ssh/erl-docker-ssh-id_rsa
RUN mkdir -p /root/code/

# Install catkin_simple
RUN git clone --branch 0.1.1 git@github.com:catkin/catkin_simple.git /root/code/catkin_simple
WORKDIR /root/code/catkin_simple
RUN { [ -d build ] && rm -rf build || mkdir build; } && \
    cd build && \
    . /opt/ros/melodic/setup.sh && \
    cmake .. && \
    make -j && \
    make install
# Done catkin_simple

## Install pybind11
RUN git clone --branch v2.2.4 https://github.com/pybind/pybind11.git /root/code/pybind11
WORKDIR /root/code/pybind11
RUN { [ -d build ] && rm -rf build/* || mkdir build; } && \
    cd build && \
    cmake ..

RUN cd build && make -j

RUN cd build && make install

## Done pybind11

RUN mkdir -p /root/code/erl_viz_ws/src
ENV CATKIN_WS  /root/code/erl_viz_ws
ENV CATKIN_SRC  /root/code/erl_viz_ws/src/
WORKDIR $CATKIN_WS
RUN cd $CATKIN_WS && . /opt/ros/melodic/setup.sh && catkin_make

RUN git clone git@bitbucket.org:ExistentialRobotics/erl_msgs.git $CATKIN_SRC/erl_msgs
RUN git clone git@bitbucket.org:ExistentialRobotics/erl_utilities.git $CATKIN_SRC/erl_utilities
RUN git clone git@bitbucket.org:ExistentialRobotics/erl_models.git $CATKIN_SRC/erl_models


COPY . $CATKIN_SRC/erl_visualization
RUN . $CATKIN_WS/devel/setup.sh && catkin_make \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON=On \
    -DPYTHON_EXECUTABLE=/usr/bin/python2.7 \
    -DPYBIND11_PYTHON_VERSION=2.7

# Make user account and starts you in your current directory
RUN chmod -R a+r /root/
RUN find /root -type d -exec chmod a+x \{} \;

ARG USER
ARG HOME
ARG UID
RUN groupadd -g "2000" "$USER" \
 && useradd --uid "${UID}" -s "/bin/bash" -c "$USER" -g "2000" -d "$HOME" "$USER" \
 && echo "$USER:$USER" | chpasswd \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER
# Commands below run as the developer user
USER $USER
ARG PWD

WORKDIR $PWD
# Environment customizations

# Entrypoint
ENTRYPOINT /bin/bash
