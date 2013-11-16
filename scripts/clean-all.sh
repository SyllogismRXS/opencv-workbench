#!/bin/bash

pushd ../ >& /dev/null

# Normal clean
./clean.sh

# Clean third-party-build
rm -rf third-party-build

popd >& /dev/null
