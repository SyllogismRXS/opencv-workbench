#!/bin/bash

##############################################
# Build yaml-cpp
##############################################

if [ ! -d ../third-party-build ];
then
    mkdir ../third-party-build
fi

if [ ! -d ../third-party-build/yaml-cpp ]; then
    echo "Downloading yaml-cpp repo..."
    
    pushd ../third-party-build >& /dev/null
    hg clone https://code.google.com/p/yaml-cpp/
    popd >& /dev/null
fi

pushd ../third-party-build/yaml-cpp >& /dev/null

if [ ! -d build ];
then
    mkdir build
fi

pushd build

cmake ..
make $@

popd >& /dev/null # build

popd >& /dev/null # yaml-cpp

