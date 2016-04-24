#!/bin/bash

if [ $# -ne 2 ]; then
    echo "usage: $0 <input-dir> <FinalWidthxFinalHeight+Xoffset+Yoffset>"
    exit;
fi

IN_DIR=$(readlink -f "$1")
OUT_DIR=$(readlink -f "${IN_DIR}/cropped")
mkdir -p ${OUT_DIR}

IMAGES=$(find ${IN_DIR} -name "*.png")

for f in $IMAGES
do
    echo "File: ${f}" 
    FILENAME=$(basename ${f})
    OUTPUT_FILENAME="${OUT_DIR}/${FILENAME}"
    convert ${f} -crop ${2} +repage "${OUTPUT_FILENAME}"    
done
