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

using namespace cv;
using namespace std;

void floodFillInner(Mat& grey, Mat& result, int obj_id, Point p, int nmean, int dev, Mat& segmented, int count);
void floodFillFromSegment(Mat &grey, Mat &segments, int obj_id, Mat &result);
