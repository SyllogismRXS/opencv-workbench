#!/bin/bash

#
# Example usage:
# ./scripts/learn-test-threshold.sh -y ./data/yaml-range-params/static-threshold.yaml -f ./data/scenarios/threshold-train-test.yaml
#

OUT_DIR_NAME=$(date +"%Y-%m-%d_%H-%M-%S")
OUT_DIR="/home/syllogismrxs/Documents/Thesis/results/${OUT_DIR_NAME}"

YAML_PARAMS_FILE="empty"
YAML_VIDEO_FILES="empty"
K_FOLDS="3"

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
        -k|--kfolds)
            K_FOLDS="$2"
            shift # past argument
            ;;
        -f|--file)
            YAML_VIDEO_FILES="$2"
            shift # past argument
            ;;
        -y|--yaml)
            YAML_PARAMS_FILE="$2"
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

if [ ! -f ${YAML_PARAMS_FILE} ]; then
    echo "YAML Params file doesn't exist: ${YAML_PARAMS_FILE}. Use -y"
    exit -1;
fi

mkdir -p ${OUT_DIR}

# Copy yaml file to results directory
cp ${YAML_PARAMS_FILE} ${OUT_DIR}

if [ ! -f ${YAML_VIDEO_FILES} ]; then
    echo "YAML Sonar/Video file list doesn't exist: ${YAML_VIDEO_FILES}. Use -f"
    exit -1;
fi

# Generate the k-folds scenarios
~/repos/opencv-workbench/bin/k-fold -y ${YAML_VIDEO_FILES} -o ${OUT_DIR} -s 100 -k ${K_FOLDS}

# Put the names of the video files in an array. The k-fold program, puts the
# file names in ${YAML_VIDEO_FILES} into a simple text file at
# ${OUT_DIR}/files.txt
VIDEO_FILES_ARRAY=()

while IFS='' read -r line || [[ -n "$line" ]]; do    
    VIDEO_FILES_ARRAY+=($line)
done < "${OUT_DIR}/files.txt"

RANGES_OUT_DIR="${OUT_DIR}/ranges"
mkdir -p ${RANGES_OUT_DIR}


# Expand the Threshold parameter file
~/repos/opencv-workbench/bin/param-range -y ${YAML_PARAMS_FILE} -o ${RANGES_OUT_DIR}

# Get list of files that were created
RANGES_FILES=$(find ${RANGES_OUT_DIR} -name "*.yaml")

# for each fold:
# Sweep across all video frames designated for training
# Compute ROC curve
# Determine "optimal" threshold
# Try threshold on validation set
#end fold

# Try threshold on final test set (one time)

#RUN_DETECTOR_EXEC=$(readlink -f ./bin/run-detector)
RUN_DETECTOR_EXEC="/home/syllogismrxs/repos/opencv-workbench/bin/run-detector"

# For each fold
FOLD_DIRS=$(find ${OUT_DIR} -name "fold-*")
for FOLD_DIR in $FOLD_DIRS
do    
    TRACKS_OUT_DIR="${FOLD_DIR}/tracks"
    mkdir -p ${TRACKS_OUT_DIR}
    
    # Sweep threshold params across all video frames
    for RANGE_FILE in $RANGES_FILES
    do
        for VIDEO_FILE in "${VIDEO_FILES_ARRAY[@]}"
        do
            # Get the K-folds file for this video file
            filename=$(basename "$VIDEO_FILE")
            base_no_ext="${filename%.*}"
            K_FOLDS_FILE="${FOLD_DIR}/$base_no_ext.frame_types.yaml"

            CMD="${RUN_DETECTOR_EXEC} -f ${VIDEO_FILE} -p relative_detector -y $RANGE_FILE -h -o ${TRACKS_OUT_DIR} -t -g thresh -m learning -k ${K_FOLDS_FILE}"
            echo $CMD
            ${CMD}
        done
    done
    
    # Aggregate data and compute ROC curve
    # Compute "optimal" threshold
    /home/syllogismrxs/repos/opencv-workbench/bin/fold-aggregate -d ${TRACKS_OUT_DIR} -o ${TRACKS_OUT_DIR} -f roc.csv -p static_threshold
    
    # Compute ROC curve, Determine "optimal" threshold
    /home/syllogismrxs/repos/opencv-workbench/scripts/roc.sh ${TRACKS_OUT_DIR}/roc.csv
    
    for VIDEO_FILE in "${VIDEO_FILES_ARRAY[@]}"
    do
        # Get the K-folds file for this video file
        filename=$(basename "$VIDEO_FILE")
        base_no_ext="${filename%.*}"
        K_FOLDS_FILE="${FOLD_DIR}/$base_no_ext.frame_types.yaml"

        # Try threshold on validation set
        echo "============="
        echo "Validating"
        CMD="${RUN_DETECTOR_EXEC} -f ${VIDEO_FILE} -p relative_detector -y ${TRACKS_OUT_DIR}/validate.yaml -h -o ${FOLD_DIR} -t -g thresh -m validating -k ${K_FOLDS_FILE}"
        echo $CMD
        ${CMD}
    done        
done

# Try "optimal" threshold on final test set
# Compute average of all winning thresholds
# write to yaml file

# for VIDEO_FILE in "${VIDEO_FILES_ARRAY[@]}"
# do
#     # Get the K-folds file for this video file (any one of the folds works,
#     # since they all have the same final "test set" 
#     filename=$(basename "$VIDEO_FILE")
#     base_no_ext="${filename%.*}"
# 
#     FOLD_DIR=$(find ${OUT_DIR} -name "fold-0")
#     K_FOLDS_FILE="${FOLD_DIR}/$base_no_ext.frame_types.yaml"    
#     
#     # Try threshold on validation set
#     echo "============="
#     echo "Testing"
#     CMD="${RUN_DETECTOR_EXEC} -f ${VIDEO_FILE} -p relative_detector -y ${OUT_DIR}/test.yaml -h -o ${OUT_DIR} -t -g thresh -m testing -k ${K_FOLDS_FILE}"
#     echo $CMD
#     ${CMD}
# done        
