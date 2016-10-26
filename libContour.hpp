#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector> 
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
void myFindContours(cv::Mat&, cv::Mat&);
void colorize(cv::Mat&, cv::Mat&);
void make_sticky(cv::Mat&,cv::Mat&,cv::Mat&);
