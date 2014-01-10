#!/bin/bash

opencv_createsamples -img $1 -num 10 -vec samples.vec -maxidev 10 -maxxangle 0.5 -maxyangle 0.5 -maxzangle 0.5 -w 60 -h 60

~/repos/opencv-workbench/bin/vec-img-display samples.vec 24 24


