ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE}

COPY ./sources.list /etc/apt

WORKDIR /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt update -y && apt install -y ubuntu-desktop-minimal  # time-consuming
RUN apt install -y \
    software-properties-common lsb-release wget vim locales apt-utils \
    python3 python3-pip python3-dev python3-setuptools python3-wheel git \
    sudo zsh ca-certificates curl iproute2 libpcre3-dev libpcre2-dev libtinyxml2-dev

# Add apt repositories
# cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
# git
RUN add-apt-repository ppa:git-core/ppa -y
# clang-format
RUN wget https://apt.llvm.org/llvm.sh && chmod u+x llvm.sh && DEBIAN_FRONTEND=noninteractive ./llvm.sh 17 && rm llvm.sh

# install packages
RUN apt update -y && apt install -y kitware-archive-keyring && apt install -y \
    cmake build-essential ccache clang-format-17 && apt upgrade -y && \
    ln -s /usr/bin/clang-format-17 /usr/bin/clang-format

# cppcheck
RUN git clone https://github.com/danmar/cppcheck.git && cd cppcheck && git checkout 2.14.1 && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release -DUSE_BOOST=ON -DUSE_LIBCXX=ON -DUSE_MATCHCOMPILER=ON \
      -DHAVE_RULES=ON -DBUILD_SHARED_LIBS=OFF -DFILESDIR=/usr/share/cppcheck -DUSE_BUNDLED_TINYXML2=OFF && \
    make -j$(nproc) && make install && cd /root && rm -rf cppcheck

# git credential manager
RUN wget https://github.com/git-ecosystem/git-credential-manager/releases/download/v2.4.1/gcm-linux_amd64.2.4.1.deb && \
    dpkg -i gcm-linux_amd64.2.4.1.deb && \
    rm gcm-linux_amd64.2.4.1.deb

# Kubernetes
RUN curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl" && \
    install -o root -g root -m 0755 kubectl /usr/local/bin/kubectl

# set locale
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# clean
RUN apt-get clean  && rm -rf /var/lib/apt/lists/*

ENV TZ="America/Los_Angeles"
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# reset DEBIAN_FRONTEND
ENV DEBIAN_FRONTEND=
