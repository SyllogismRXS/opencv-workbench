#!/bin/bash

# Arguments:
# $1 - top-level directory of annotated xml files
# $2 - config file for require tests

if [ "$#" -ne 1 ]; then
    echo "usage: $0 <input-dir>"
    exit
fi 

IN_DIR=$1
echo "--------------------------------------------------------------------"
echo "                 Finding Relevant Video Files"
echo 
echo "Input directory: ${IN_DIR}"
TRUTH_DATA=$(find $1 -name "*truth.xml")

TRUTH_DATA_ARRAY=()
VIDEO_FILES_ARRAY=()
TRACK_DATA_ARRAY=()

for f in $TRUTH_DATA
do
    # Get the 
    #filename=$(basename "$f")
    #no_ext="${filename%.truth.*}"
    no_ext="${f%.truth.*}"
    
    video_file="${no_ext}.avi"    
    if [ -e ${video_file} ]; then        
        track_file="${no_ext}.track.xml"
        
        TRUTH_DATA_ARRAY=( "${TRUTH_DATA_ARRAY[@]}" "$f")
        VIDEO_FILES_ARRAY=( "${VIDEO_FILES_ARRAY[@]}" "${video_file}")
        TRACK_DATA_ARRAY=( "${TRACK_DATA_ARRAY[@]}" "${track_file}")        
    fi
done

# Convert sonar files to avi files (32-bit only)
# Assume this is done for now

# Recursively find all .xml annotated files (associated with avi file)


# Determine which detectors are going to be tested

# Process each video file with detectors
for i in "${VIDEO_FILES_ARRAY[@]}"
do
    echo "==========================================================="
    echo "Processing video file:"
    echo  "${i}"
    echo "-----------------------------------------------------------"
    ~/repos/opencv-workbench/bin/run-detector ${i}
done

# Compare detectors results with hand annotated results

# Plot results / Generate tables
