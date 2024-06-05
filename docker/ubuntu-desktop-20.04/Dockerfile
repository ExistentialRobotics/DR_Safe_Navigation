ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE}

COPY ./sources.list /etc/apt
ENV DEBIAN_FRONTEND noninteractive

RUN apt update -y
RUN apt install -y \
    software-properties-common lsb-release wget vim locales apt-utils \
    python3 python3-pip python3-dev python3-setuptools python3-wheel \
    sudo zsh ca-certificates curl iproute2 ubuntu-desktop-minimal

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
    cmake build-essential cppcheck ccache clang-format-17 && apt upgrade -y

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
