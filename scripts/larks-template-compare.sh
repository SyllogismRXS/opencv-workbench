#!/bin/bash

pushd ../bin >& /dev/null

OUTPUT_FILE="$(date +%Y_%m_%d-%H.%M.%S)-larks-template-compare.txt"
touch $OUTPUT_FILE

for i in `seq -1 5`;
do
    ./larks-test ~/Dropbox/video/target.avi ../data/images/query.png ../data/label/target.avi.scuba_face.label $i 0.5 0.5 0.6 >> $OUTPUT_FILE 2>&1
done 

popd >& /dev/null
