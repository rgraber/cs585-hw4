#!/bin/sh
all: background_sub.cpp
	g++ -g -O3 -std=c++11 -o subtract background_sub.cpp libContour.cpp boundary.cpp floodfill.cpp segmentation.cpp  -I/usr/local/include/opencv -I/usr/local/include/ -L/usr/local/lib  -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab  -lopencv_calib3d -lopencv_features2d -lopencv_highgui  -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -fpermissive
clean:
	$(RM) bats
