# syntax=docker/dockerfile:1
# install ubuntu base with updates
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Rome"
RUN apt-get update
RUN apt-get -y upgrade
RUN apt-get -y install apt-utils
RUN apt-get -y install wget curl 
RUN apt-get -y install tzdata nano bc tmux
RUN apt-get -y install locales

# generate locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 

# Replace sh with bash                                                         
RUN rm /bin/sh && ln -s bash /bin/sh

# install dependencies for YOCTO
RUN apt-get install -y gawk wget git-core diffstat unzip texinfo gcc-multilib build-essential chrpath socat cpio python python3 python3-pip python3-pexpect xz-utils debianutils iputils-ping python3-git python3-jinja2 libegl1-mesa libsdl1.2-dev pylint3 xterm rsync curl libncurses-dev
RUN apt-get install -y lz4 zstd
##RUN apt-get install -y  gcc-multilib g++-multilib
RUN apt-get install -y device-tree-compiler

# download and install repo
RUN curl https://storage.googleapis.com/git-repo-downloads/repo > /usr/local/bin/repo
RUN chmod a+x /usr/local/bin/repo

# some packages needs kernel sourecs/headers
RUN apt-get -y install linux-headers-5.4.0-84-generic
RUN mkdir -p /lib/modules/$(uname -r)/
RUN ln -sf /usr/src/linux-headers-5.4.0-84-generic /lib/modules/$(uname -r)/build

# configure userspace (ase)
USER root
RUN useradd -d /home/ase -m -p ase -U -s /bin/bash ase
RUN chown -R ase:ase /home/ase
USER ase
WORKDIR /home/ase

# configure git
RUN git config --global user.name "ASE development tool"
RUN git config --global user.email "ase@densitron.com"
RUN git config --list



# Create the YOCTO working directory for the bind mount
RUN mkdir imx-yocto-bsp
WORKDIR imx-yocto-bsp


### following section is not still needed. Use the command
#   $ ./init-data.sh
#   to build the working directory

##COPY --chown=ase ./bsp-root .
##COPY --chown=ase ./sources  ./sources

##RUN mkdir -p build/
##COPY --chown=ase ./conf ./build/conf
##RUN chmod 444 ./build/conf/*

##RUN ./imx-setup-release.sh -b build
##COPY --chown=ase conf/* build/conf/
##RUN chmod 444 build/conf/local.conf



# Add entrypoint script
COPY --chown=ase entrypoint.sh /home/ase/entrypoint.sh
RUN chmod +x /home/ase/entrypoint.sh

ENTRYPOINT ["/home/ase/entrypoint.sh"]
CMD ["/bin/bash"]


