#!/bin/bash

pushd ../bin >& /dev/null

OUTPUT_FILE="$(date +%Y_%m_%d-%H.%M.%S)-larks-template-compare.txt"
touch $OUTPUT_FILE

for i in `seq -1 5`;
do
    ./larks-test ~/Dropbox/video/diver-swim.avi ~/repos/opencv-workbench/data/images/diver-sonar.png ~/repos/opencv-workbench/data/label/diver-swim.avi.diver.label $i >> $OUTPUT_FILE 2>&1
    #./larks-test $i >> $OUTPUT_FILE 2>&1
done 

popd >& /dev/null
