#!/bin/bash
usage()
{
cat << EOF
usage: sudo $0 <linux-variant>
This script installs all dependencies for opencv-workbench

EOF
}

#Require the script to be run as root
if [[ $(/usr/bin/id -u) -ne 0 ]]; then
    echo "This script must be run as root because libraries will be installed."
    usage
    exit
fi

# Grab username of caller for later
ORIGINAL_USER=$(who am i | awk '{print $1}')

if [ ! -d ../third-party-build ];
then
su $ORIGINAL_USER -m -c 'mkdir ../third-party-build'
fi

# Install dependencies through the package manager first:
./install-bin-deps.sh

# -----------------------------------------------------------------------------
# Build yaml-cpp
# -----------------------------------------------------------------------------
su $ORIGINAL_USER -m -c './build-yaml-cpp.sh'
./build-yaml-cpp.sh install
ldconfig

# -----------------------------------------------------------------------------
# Build OpenCV
# -----------------------------------------------------------------------------
su $ORIGINAL_USER -m -c './build-opencv.sh'
./build-opencv.sh install
ldconfig
