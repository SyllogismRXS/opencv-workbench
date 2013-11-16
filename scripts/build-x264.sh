#!/bin/bash

##############################################
# Build x264
##############################################

if [ ! -d ../third-party-build ];
then
    mkdir ../third-party-build
fi

if [ ! -d ../third-party-build/x264 ]; then
    echo "Downloading x264 repo..."
    
    pushd ../third-party-build >& /dev/null
    git clone git://git.videolan.org/x264.git
    popd >& /dev/null
fi

pushd ../third-party-build/x264 >& /dev/null

./configure --enable-static --enable-shared --disable-asm
make $@

popd >& /dev/null # build

popd >& /dev/null # x264

