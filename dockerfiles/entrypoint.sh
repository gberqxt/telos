#!/bin/bash

   
# Run the setup script
chown ase:ase /home/ase/imx-yocto-bsp
cd /home/ase/imx-yocto-bsp
## source ./imx-setup-release.sh -b build
## source ./aaeon-imx-setup-release.sh -b imx8p_build


# Execute the command passed to docker run (or default to bash)
exec "$@"
