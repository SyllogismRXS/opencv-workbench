#!/bin/bash

# Suggested directory setup:
# ~/Documents/scuba-faces-db/originals
# ~/Documents/scuba-faces-db/cropped
# ~/Documents/scuba-faces-db/processing
# cd into ~/Documents/scuba-faces-db/processing and run
# ~/repos/opencv-workbench/scripts/samples-generation.sh
# which generates all training and testing samples
#
# Run ~/repos/opencv-workbench/scripts/training.sh
# which trains the face detector
#
# 

WIDTH=60
HEIGHT=60
NPOS=500

# Create samples description file for positive images
find ../cropped -name '*.JPG' -exec identify -format '%i 1 0 0 %w %h' \{\} \; > positives.dat

# Create negative samples data file
find ../../negatives -name '*.jpg' > negatives.dat

# Second run...
perl ~/repos/opencv-workbench/scripts/createtrainsamples.pl positives.dat negatives.dat samples $NPOS "opencv_createsamples  -bgcolor 0 -bgthresh 0 -maxxangle 0.01 -maxyangle 0.01 maxzangle 0.01 -maxidev 20 -w $WIDTH -h $HEIGHT"
# First run...
#perl ~/repos/opencv-workbench/scripts/createtrainsamples.pl positives.dat negatives.dat samples 1000 "opencv_createsamples  -bgcolor 0 -bgthresh 0 -maxxangle 1.1 -maxyangle 1.1 maxzangle 0.5 -maxidev 40 -w $WIDTH -h $HEIGHT"
#perl ~/repos/opencv-workbench/scripts/createtrainsamples.pl positives.dat negatives.dat samples 7000 "opencv_createsamples  -bgcolor 0 -bgthresh 0 -maxxangle 1.1 -maxyangle 1.1 maxzangle 0.5 -maxidev 40 -w $WIDTH -h $HEIGHT"

# Merge individual vector files
find samples/ -name '*.vec' > samples.dat
~/repos/opencv-workbench/bin/mergevec samples.dat samples.vec -w $WIDTH -h $HEIGHT

# Create test samples
perl ~/repos/opencv-workbench/scripts/createtestsamples.pl positives.dat negatives.dat tests 1000 "opencv_createsamples -bgcolor 0 -bgthresh 0 -maxxangle 1.1 -maxyangle 1.1 -maxzangle 0.5 maxidev 40 -w $WIDTH -h $HEIGHT"

# Create test samples data file
find tests/ -name 'info.dat' -exec cat \{\} \; > tests.dat

## # Display contents of final vector file
## ~/repos/opencv-workbench/bin/vec-img-display samples.vec $WIDTH $HEIGHT

## #opencv_createsamples -img $1 -num 10 -bg negatives.data -vec samples.vec -maxidev 10 -maxxangle 0.5 -maxyangle 0.5 -maxzangle 0.5 -w 60 -h 60
## 
## ## Many from one
## opencv_createsamples -img $1 -num 10 -bg negatives.dat -vec samples.vec -maxxangle 0.6 -maxyangle 0 -maxzangle 0.3 -maxidev 100 -bgcolor 0 -bgthresh 0 -w $WIDTH -h $HEIGHT
## 
## # Many from many
## #opencv_createsamples -info samples.dat -vec samples.vec -w $WIDTH -h $HEIGHT
## 
## #opencv_createsamples -vec samples.vec -w $WIDTH -h $HEIGHT
## 
## ## Create test samples
## #opencv_createsamples -img $1 -num 10 -bg negatives.dat -info test.dat -maxxangle 0.6 -maxyangle 0 -maxzangle 0.3 -maxidev 100 -bgcolor 0 -bgthresh 0
