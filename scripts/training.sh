#!/bin/bash

WIDTH=40
HEIGHT=40
NPOS=500

#script -c "opencv_haartraining -data haarcascade -vec samples.vec -bg negatives.dat -nstages 20 -nsplits 2 -minhitrate 0.999 -maxfalsealarm 0.5 -npos $NPOS -nneg 3019 -w $WIDTH -h $HEIGHT -nonsym -mem 1500 -mode ALL" training-output.txt
#opencv_haartraining -data haarcascade -vec samples.vec -bg negatives.dat -nstages 20 -nsplits 2 -minhitrate 0.999 -maxfalsealarm 0.5 -npos $NPOS -nneg 3019 -w $WIDTH -h $HEIGHT -nonsym -mem 1500 -mode ALL | tee training-output.txt 2>&1
opencv_haartraining -data haarcascade -vec samples.vec -bg negatives.dat -nstages 20 -nsplits 2 -minhitrate 0.999 -maxfalsealarm 0.5 -npos $NPOS -nneg 3019 -w $WIDTH -h $HEIGHT -nonsym -mem 2000 -mode ALL
#opencv_haartraining -data haarcascade -vec samples.vec -bg negatives.dat -nstages 20 -nsplits 2 -minhitrate 0.999 -maxfalsealarm 0.5 -npos $NPOS -nneg 3019 -w $WIDTH -h $HEIGHT -nonsym -mem 1500 -mode ALL
#opencv_haartraining -data haarcascade -vec samples.vec -bg negatives.dat -nstages 20 -nsplits 2 -minhitrate 0.999 -maxfalsealarm 0.5 -npos $NPOS -nneg 3019 -w $WIDTH -h $HEIGHT -nonsym -mem 1500 -mode ALL > training-output.txt 2>&1
#opencv_haartraining -data haarcascade -vec samples.vec -bg negatives.dat -nstages 20 -nsplits 2 -minhitrate 0.999 -maxfalsealarm 0.5 -npos $NPOS -nneg 3019 -w $WIDTH -h $HEIGHT -nonsym -mem 1500 -mode ALL > training-output.txt 2>&1

# Convert to XML
~/repos/opencv-workbench/bin/convert_cascade --size="$WIDTHx$HEIGHT" haarcascade haarcascade.xml

# Test the performance
#script -c "opencv_performance -data haarcascade.xml -info tests.dat -ni" performance-output.txt
opencv_performance -data haarcascade.xml -info tests.dat -ni > performance-output.txt
