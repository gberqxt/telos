# TLOS YOCTO PROJECT

This repo contains docker configuration files and the densitron meta layer for Telos YOCTO project using NXP iMX8 pico board by AAEON.

How to setup:


## Telos YOCTO Environment Setup

Here the instructions for making the image

### Restore environment and working set

Developing machine:
```
  ssh -X gianluca@192.168.40.60
  cd ~/workspace/Densitron/test-1
```

This repo contains a docker setup for compiling the iMX8 image.


In the development directory start the docker session
```
docker compose up -d
docker exec -it telos-inf1000a bash
```

It is convenient to use `tmux` in order to have a running session even on closing the `ssh` connection.

> ***IMPORTANT:***     After running docker, the new working directory is
>              `ase@telos-inf1000a:~/imx-yocto-bsp$`


### Restore project environment

This step must be executed just once.
Skip this session if you already have set it up.

Download Yocto BSP with kernel 5.15.71:
```
repo init -u https://github.com/nxp-imx/imx-manifest \
          -b imx-linux-kirkstone -m imx-5.15.71-2.2.2.xml

repo sync
```

Environment setup:
```
DISTRO=fsl-imx-wayland MACHINE=imx8mpevk source imx-setup-release.sh -b imx8p_build
```

On running this command the environment setup is created and activated.

exit from environment running
```
exit
```
or continue into the just created environment.



### Build code environment

Start YOCTO environment:
```
docker exec -it telos-inf1000a bash

. ./setup-environment imx8p_build
```


Rebuild

```
bitbake imx-image-multimedia
```




