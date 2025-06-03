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
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0-linux-x86_64.sh &&\
    chmod +x cmake-3.24.0-linux-x86_64.sh && \
    ./cmake-3.24.0-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.24.0-linux-x86_64.sh

# git
RUN add-apt-repository ppa:git-core/ppa -y

# install packages
RUN apt update -y && apt install -y \
    build-essential ccache git && apt upgrade -y

# git credential manager
RUN wget https://github.com/git-ecosystem/git-credential-manager/releases/download/v2.4.1/gcm-linux_amd64.2.4.1.deb && \
    dpkg -i gcm-linux_amd64.2.4.1.deb && \
    rm gcm-linux_amd64.2.4.1.deb

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
