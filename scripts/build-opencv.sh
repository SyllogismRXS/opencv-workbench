#!/bin/bash

##############################################
# Build OpenCV
##############################################

if [ ! -d ../third-party-build ];
then
    mkdir ../third-party-build
fi

if [ ! -d ../third-party-build/opencv ]; then
    echo "Extracting opencv repo..."
    pushd ../third-party-build >& /dev/null
    tar xvf ../third-party-src/opencv-2*
    popd >& /dev/null

    #echo "Downloading opencv repo..."
    #
    #pushd ../third-party-build >& /dev/null
    #git clone https://github.com/Itseez/opencv.git
    #popd >& /dev/null
fi

pushd ../third-party-build/opencv-2* >& /dev/null

if [ ! -d build ];
then
    mkdir build
fi

pushd build

cmake ..
make $@

popd >& /dev/null # build

popd >& /dev/null # opencv

