#!/bin/bash

WIDTH=60
HEIGHT=60
NPOS=500

opencv_haartraining -data haarcascade -vec samples.vec -bg negatives.dat -nstages 20 -nsplits 2 -minhitrate 0.999 -maxfalsealarm 0.5 -npos $NPOS -nneg 3019 -w $WIDTH -h $HEIGHT -nonsym -mem 1500 -mode ALL | tee training-output.txt 2>&1

# Convert to XML
~/repos/opencv-workbench/bin/convert_cascade --size="$WIDTHx$HEIGHT" haarcascade haarcascade.xml

# Test the performance
opencv_performance -data haarcascade.xml -info tests.dat -ni | tee performance-output.txt 2>&1
