//opencv libraries
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

string frameNumberToTime(int idx, int fps=10);
Rect rectFromMask(Mat& mask);
void mergeHistory(vector<Mat> history, Mat& dst);
vector<string> filesInFolder(string& dirpath);
string type2str(int type);
void createMatchedMask(string img_file, string template_file, Mat& mask);

Point getLowestPoint(Vector<Point> in);
Point getHighestPoint(Vector<Point> in);
Point getRightMostPoint(Vector<Point> in);
Point getLeftMostPoint(Vector<Point> in);
#endif /* UTILS_HPP_ */
