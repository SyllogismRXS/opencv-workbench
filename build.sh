#!/bin/bash

# You'll want to remove the following programs first...
# sudo apt-get remove ffmpeg x264 libx264-dev

mkdir ./build -p
pushd build >& /dev/null

cmake ..

make
BUILD_RESULT=$?

popd >& /dev/null

# Check status of make build command to see if there were any errors
# and ...
# If arguments are supplied to the build script, then this could mean
# that the user either wants to install the project or execute a program
if [ ${BUILD_RESULT} -eq 0 ] && [ $# -gt 0 ]; then
    # The user wants to install the project
    if [ $1 == "install" ]; then
        make install
    else
        # The user wants to execute a script in the bin directory
        # Move the bin directory and pass the command to call
        # Example call: ./build.sh ./hello-world
        pushd ./bin >& /dev/null
        #$1
        $1 ~/Documents/Thesis/data/sonar-avi/2014-01-24-ME-sonar-only-diver/2014_01_24_15_24_06.avi
        popd >& /dev/null
    fi
fi

# Test Calls:
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label -1 0.01 0.01 0.08
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label -1 0.18 0.01 0.20

# Method 0 Calls
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 0 36780200 4598920 82769400
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 0 78170500 1182950 90000000

# Method 1 Calls
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 1 36780200 4598920 82769400
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 1 0.126967 0.021846 0.345426
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 1 0.323581 0.0043692 0.345427

# Method 2 Calls
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label -1 334442000 11563300 450075000

# Method 3 Calls
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 3 0.803431 0.010495 0.958381

# Method 4 Calls
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 4 0.2831 0.053019 0.813291
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 4 29681700 4593300 75614700

# Method 5 Calls
#./bin/larks-test ~/Dropbox/video/target.avi ./data/images/query.png ./data/label/target.avi.scuba_face.label 5 0.2831 0.053019 0.813291

