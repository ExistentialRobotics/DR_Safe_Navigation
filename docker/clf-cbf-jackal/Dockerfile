FROM erl/ros-noetic:12.3.2-cudnn9-devel-ubuntu20.04

RUN wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \ | gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null
RUN echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list
RUN apt update && apt upgrade -y
RUN apt install -y ros-noetic-pybind11-catkin \
    python3-catkin-tools \
    libcgal-dev \
    liblapacke-dev \
    python3-pytest \
    nlohmann-json3-dev \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-tf-conversions \
    ros-noetic-rviz \
    python3-numpy \
    python3-scipy \
    libboost-all-dev \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-position-controllers \
    ros-noetic-map-server \
    ros-noetic-hector-mapping \
    ros-noetic-tf2-sensor-msgs \
    intel-basekit-2024.1 \
    ccache \
    xterm
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-jackal*
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-rqt*

WORKDIR /root

RUN git clone https://gitlab.com/libeigen/eigen.git && cd eigen && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make install && cd /root && rm -rf eigen

RUN git clone https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp && git checkout 0e6e28d && \
    mkdir build && cd build && cmake .. -DYAML_BUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . --target install -j$(nproc) && cd /root && rm -rf yaml-cpp

RUN wget https://github.com/abseil/abseil-cpp/releases/download/20240116.1/abseil-cpp-20240116.1.tar.gz && \
    tar -xf abseil-cpp-20240116.1.tar.gz && rm abseil-cpp-20240116.1.tar.gz && cd abseil-cpp-20240116.1 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="${CXXFLAGS} -DNDEBUG" -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON -DABSL_PROPAGATE_CXX_STD=ON && \
    make -j`nproc` && make install && cd /root && rm -rf abseil-cpp-20240116.1

RUN wget https://github.com/fmtlib/fmt/archive/refs/tags/9.1.0.tar.gz && tar -xf 9.1.0.tar.gz && rm 9.1.0.tar.gz && \
    cd fmt-9.1.0 && mkdir build && cd build && \
    cmake .. -DFMT_DOC=OFF -DFMT_TEST=OFF -DFMT_INSTALL=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON && \
    make -j`nproc` && make install && cd /root && rm -rf fmt-9.1.0

RUN git clone https://github.com/robotics-upo/lightsfm && cd lightsfm && make && make install && cd /root && rm -rf lightsfm

RUN pip install cvxpy vedo==2024.5.2 && pip install -U numpy==1.23.5

RUN apt-get clean && rm -rf /var/lib/apt/lists/*
