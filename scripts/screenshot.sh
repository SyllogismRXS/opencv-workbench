#!/bin/bash
# An array of window names to take screenshots of
WINDOWS=(
    "SonarImage"
    "Gray"
    "Median"
    "Thresh"
    "Erode"
    "Dilate"
    "FrameBlobs"
    "BlobTracks"
    "Consolidate"
    "Objects"
)

if [ $# -lt 1 ]; then
    echo "Missing output directory"
    exit;
fi

OUT_DIR="$1"
mkdir -p ${OUT_DIR}
OUT_DIR=$(readlink -f ${OUT_DIR})

count=0
for i in "${WINDOWS[@]}" 
do
    :        
    wmctrl -a ${i}
    STATUS=$?
    if [ $STATUS -eq 0 ]; then    
        scrot -u "${OUT_DIR}/${count}-${i}.png"
        sleep 0.3
    else
        echo "Window doesn't exist: ${i}"
    fi
    
    ((count++))
done
