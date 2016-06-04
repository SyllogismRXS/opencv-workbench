#!/bin/bash

#
# Example usage:
# ./scripts/learn-test-velocity -y ./data/yaml-range-params/static-threshold.yaml -f ./data/scenarios/threshold-train-test.yaml
#

OUT_DIR_NAME=$(date +"%Y-%m-%d_%H-%M-%S")
OUT_DIR="/home/syllogismrxs/Documents/Thesis/results/${OUT_DIR_NAME}"

YAML_PARAMS_FILE="empty"
YAML_VIDEO_FILES="empty"
K_FOLDS="3"
SWEEP_PARAM="empty"
NEG_TO_POS_RATIO="3"
HIDE_WINDOWS=" "

OPENCV_WORKBENCH_ROOT="/home/syllogismrxs/repos/opencv-workbench"

run_cmd()
{
    CMD=$1
    echo "===================================================================="
    echo $CMD
    echo "===================================================================="
    ${CMD}    
}

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
        -r|--ratio)
            NEG_TO_POS_RATIO="$2"
            shift # past argument
            ;;
        -s|--sweep)
            SWEEP_PARAM="$2"
            shift # past argument
            ;;
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
        -h|--hide_windows)
            HIDE_WINDOWS="-h"
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

if [ ! -f ${YAML_VIDEO_FILES} ]; then
    echo "YAML Sonar/Video file list doesn't exist: ${YAML_VIDEO_FILES}. Use -f"
    exit -1;
fi

if [ "$SWEEP_PARAM" == "empty" ]; then
    echo "Specify parameter to sweep for ROC plots with -s"
    exit -1;
fi

if [[ "$SWEEP_PARAM" != "static_threshold" && "$SWEEP_PARAM" != "ratio_threshold" && "$SWEEP_PARAM" != "gradient_threshold" && "$SWEEP_PARAM" != "min_velocity_threshold" ]]; then
    echo "Probably not a valid SWEEP_PARAM. Try again or add it here. -s"
    exit -1;
fi

mkdir -p ${OUT_DIR}

# Copy yaml file to results directory
cp ${YAML_PARAMS_FILE} ${OUT_DIR}

# Generate the k-folds scenarios
run_cmd "${OPENCV_WORKBENCH_ROOT}/bin/k-fold -y ${YAML_VIDEO_FILES} -o ${OUT_DIR} -s 100 -k ${K_FOLDS}"

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
run_cmd "${OPENCV_WORKBENCH_ROOT}/bin/param-range -y ${YAML_PARAMS_FILE} -o ${RANGES_OUT_DIR}"

# Get list of files that were created
RANGES_FILES=$(find ${RANGES_OUT_DIR} -name "*.yaml")

RUN_DETECTOR_EXEC="${OPENCV_WORKBENCH_ROOT}/bin/run-detector"

###############################################################################
# Sweep over the yaml range params and "learn" the best threshold
# For each fold
###############################################################################
FOLD_DIRS=$(find ${OUT_DIR} -name "fold-*")
for FOLD_DIR in $FOLD_DIRS
do        
    echo "Fold_dir: ${FOLD_DIR}"
    
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

            run_cmd "${RUN_DETECTOR_EXEC} -f ${VIDEO_FILE} -p relative_detector -y $RANGE_FILE ${HIDE_WINDOWS} -o ${TRACKS_OUT_DIR} -t -g detection -m learning -k ${K_FOLDS_FILE}"
            #${CMD} >> "${OUT_DIR}/detector.txt" 2>&1
            #${CMD}
        done
    done
    
    # Aggregate data and compute ROC curve
    # Compute "optimal" threshold
    # Generate validate.yaml file
    run_cmd "${OPENCV_WORKBENCH_ROOT}/bin/fold-aggregate -d ${TRACKS_OUT_DIR} -o ${TRACKS_OUT_DIR} -r ${RANGES_OUT_DIR} -g CLASSIFIER -f roc.csv -s ${SWEEP_PARAM}"
    
    # Draw ROC Curve
    run_cmd "${OPENCV_WORKBENCH_ROOT}/scripts/roc.sh ${TRACKS_OUT_DIR}/roc.csv"        
    
    ###########################################################################
    # Cross Validation for this fold
    ###########################################################################
    for VIDEO_FILE in "${VIDEO_FILES_ARRAY[@]}"
    do
        # Get the K-folds file for this video file
        filename=$(basename "$VIDEO_FILE")
        base_no_ext="${filename%.*}"
        K_FOLDS_FILE="${FOLD_DIR}/$base_no_ext.frame_types.yaml"

        # Try threshold on validation set
        echo "============="
        echo "Validating"
        run_cmd "${RUN_DETECTOR_EXEC} -f ${VIDEO_FILE} -p relative_detector -y ${TRACKS_OUT_DIR}/validate.yaml ${HIDE_WINDOWS} -o ${FOLD_DIR} -t -g detection -m validating -k ${K_FOLDS_FILE}"
        #${CMD} >> "${OUT_DIR}/detector.txt" 2>&1        
    done        
done

# Compute Average ROC Curve From K-Folds (output test.yaml file)
run_cmd "${OPENCV_WORKBENCH_ROOT}/bin/roc-average -s ${SWEEP_PARAM} -d ${OUT_DIR} -o ${OUT_DIR} -r ${RANGES_OUT_DIR}"

# Plot the Average ROC Curve (Full Points)
run_cmd "${OPENCV_WORKBENCH_ROOT}/scripts/avg-roc.sh ${OUT_DIR}/avg-roc.csv"

# Plot the Average ROC Curve (Decimated by 10)
run_cmd "${OPENCV_WORKBENCH_ROOT}/scripts/avg-roc.sh ${OUT_DIR}/avg-roc.csv 10"

###############################################################################
# Testing with the resulting "optimal" operating point
###############################################################################
for VIDEO_FILE in "${VIDEO_FILES_ARRAY[@]}"
do
    # Get the K-folds file for this video file (any one of the folds works,
    # since they all have the same final "test set" 
    filename=$(basename "$VIDEO_FILE")
    base_no_ext="${filename%.*}"

    FOLD_DIR=$(find ${OUT_DIR} -name "fold-0")
    K_FOLDS_FILE="${FOLD_DIR}/$base_no_ext.frame_types.yaml"    
    
    # Try threshold on validation set
    echo "============="
    echo "Testing"
    run_cmd "${RUN_DETECTOR_EXEC} -f ${VIDEO_FILE} -p relative_detector -y ${OUT_DIR}/test.yaml ${HIDE_WINDOWS} -o ${OUT_DIR} -t -g detection -m testing -k ${K_FOLDS_FILE}"
    #echo $CMD
    #${CMD} >> "${OUT_DIR}/detector.txt" 2>&1
    #${CMD}
done        

# Compute the final accuracy for the test sets
run_cmd "${OPENCV_WORKBENCH_ROOT}/bin/aggregate-test -d ${OUT_DIR} -o ${OUT_DIR} -g CLASSIFIER"
