#ifndef __utils_H_INCLUDED__
#define __utils_H_INCLUDED__

//opencv libraries
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
//C++ standard libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <glob.h>
#include <math.h>

using namespace cv;
using namespace std;

// segmentation
vector<Point> skeletonToContour(Mat& src, int obj_id=255);
vector<Point> floodFillAndVectorize(
        Mat& src, Mat& buffer, vector<Point> searchSpace, int found=255);
void neighborhood(vector<Point> points, vector<Point>& neighbors);
void distanceToBackground(Mat& src, Mat& dst, int obj_id=255);
Point borderPoint(Mat& src, int obj_id=255);
void skeletonize(Mat& src, Mat& dst, int obj_id=255);

// analysis
double circularity(Mat& src, int obj_id=255);
double circularity(vector<Point> object);
double orientation(Mat& src, int obj_id=255);
double orientation(vector<Point> object);
Point centroid(Mat& src, int obj_id=255);
Point centroid(vector<Point> object);
int area(Mat& src, int obj_id=255);
int area(vector<Point> object);

vector<double> curvature_dtds(vector<Point> contour);
vector<double> curvature_vss(vector<Point> contour);
vector<double> curvature_k(vector<Point> contour);

// utils
void normalize(vector<double>& vec, double a=0.0, double b=1.0);
vector<Point> foreground(Mat& src, int obj_id=255);
vector<Point> drawPointsOnLine(Mat& src, Point center, double theta);
bool inBounds(Point p, int w, int h);

#endif
