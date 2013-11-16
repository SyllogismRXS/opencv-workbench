#!/bin/bash

##############################################
# Build ffmpeg
##############################################

if [ ! -d ../third-party-build ];
then
    mkdir ../third-party-build
fi

if [ ! -d ../third-party-build/ffmpeg ]; then
    echo "Downloading ffmpeg repo..."
    
    pushd ../third-party-build >& /dev/null
    git clone git://source.ffmpeg.org/ffmpeg.git ffmpeg
    popd >& /dev/null
fi

pushd ../third-party-build/ffmpeg >& /dev/null

./configure --enable-gpl --enable-libfaac --enable-libmp3lame --enable-libopencore-amrnb --enable-libopencore-amrwb --enable-libtheora --enable-libvorbis --enable-libx264 --enable-libxvid --enable-nonfree --enable-postproc --enable-version3 --enable-x11grab
make $@

popd >& /dev/null # build

popd >& /dev/null # ffmpeg

