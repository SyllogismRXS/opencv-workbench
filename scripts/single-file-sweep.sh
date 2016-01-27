#!/bin/bash

#
# Example usage:
# ./scripts/single-file-sweep.sh -y ./data/yaml-range-params/baseline.yaml -f ~/Documents/Thesis/data/2014-01-24-ME-sonar-only-diver/2014_01_24_15_24_06.son
#

OUT_DIR_NAME=$(date +"%Y-%m-%d_%H-%M-%S")
OUT_DIR="/home/syllogismrxs/Documents/Thesis/results/${OUT_DIR_NAME}"

YAML_FILE="empty"
VIDEO_FILE="empty"

# Use > 1 to consume two arguments per pass in the loop (e.g. each
# argument has a corresponding value to go with it).
# Use > 0 to consume one or more arguments per pass in the loop (e.g.
# some arguments don't have a corresponding value to go with it such
# as in the --default example).
# note: if this is set to > 0 the /etc/hosts part is not recognized ( may be a bug )
# ./myscript.sh -e conf -s /etc -l /usr/lib /etc/hosts 
while [[ $# > 1 ]]
do
    key="$1"

    case $key in
        -f|--file)
            VIDEO_FILE="$2"
            shift # past argument
            ;;
        -y|--yaml)
            YAML_FILE="$2"
            shift # past argument
            ;;
        -o|--output_dir)
            OUT_DIR="$2/${OUT_DIR_NAME}"
            shift # past argument
            ;;
        --default)
            DEFAULT=YES
            ;;
        *)
            # unknown option
            ;;
    esac
    shift # past argument or value
done

if [ ! -f ${YAML_FILE} ]; then
    echo "YAML file doesn't exist: ${YAML_FILE}"
    exit -1;
fi

mkdir -p ${OUT_DIR}

# Copy yaml file to results directory
cp ${YAML_FILE} ${OUT_DIR}

if [ ! -f ${VIDEO_FILE} ]; then
    echo "Sonar/Video file doesn't exist: ${VIDEO_FILE}"
    exit -1;
fi

RANGES_OUT_DIR="${OUT_DIR}/ranges"
TRACKS_OUT_DIR="${OUT_DIR}/tracks"

mkdir -p ${RANGES_OUT_DIR}
mkdir -p ${TRACKS_OUT_DIR}

# Expand the Threshold parameter file
~/repos/opencv-workbench/bin/param-range -y ${YAML_FILE} -o ${RANGES_OUT_DIR}

# Get list of files that were created
RANGES_FILES=$(find ${RANGES_OUT_DIR} -name "*.yaml")


for f in $RANGES_FILES
do
    ~/repos/opencv-workbench/bin/run-detector -f ${VIDEO_FILE} -p relative_detector -y $f -h -o ${TRACKS_OUT_DIR} -t
    ## Get the 
    ##filename=$(basename "$f")
    ##no_ext="${filename%.truth.*}"
    #no_ext="${f%.truth.*}"
    #
    #video_file="${no_ext}.avi"    
    #if [ -e ${video_file} ]; then        
    #    track_file="${no_ext}.track.xml"
    #    
    #    TRUTH_DATA_ARRAY=( "${TRUTH_DATA_ARRAY[@]}" "$f")
    #    VIDEO_FILES_ARRAY=( "${VIDEO_FILES_ARRAY[@]}" "${video_file}")
    #    TRACK_DATA_ARRAY=( "${TRACK_DATA_ARRAY[@]}" "${track_file}")        
    #fi
done

# Compute aggregated metrics
~/repos/opencv-workbench/bin/aggregate -d ${TRACKS_OUT_DIR} -o ${OUT_DIR} -f roc.csv

# Plot results / Generate tables
~/repos/opencv-workbench/scripts/roc.sh "${OUT_DIR}/roc.csv"
