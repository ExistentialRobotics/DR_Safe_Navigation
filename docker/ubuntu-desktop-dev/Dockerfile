ARG BASE_IMAGE=ubuntu-desktop-20.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND noninteractive

# install vscode
RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add -
RUN add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" && \
    apt update -y && apt install -y code

# install jetbrains
RUN curl -s https://s3.eu-central-1.amazonaws.com/jetbrains-ppa/0xA6E8698A.pub.asc | gpg --dearmor | tee /usr/share/keyrings/jetbrains-ppa-archive-keyring.gpg > /dev/null
RUN echo "deb [signed-by=/usr/share/keyrings/jetbrains-ppa-archive-keyring.gpg] http://jetbrains-ppa.s3-website.eu-central-1.amazonaws.com any main" | tee /etc/apt/sources.list.d/jetbrains-ppa.list > /dev/null
RUN apt update -y && apt install -y clion pycharm-professional

# clean
RUN apt-get clean  && rm -rf /var/lib/apt/lists/*

# reset DEBIAN_FRONTEND
ENV DEBIAN_FRONTEND=
