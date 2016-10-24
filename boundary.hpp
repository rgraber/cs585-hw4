/*
 * boundary.hpp
 *
 *  Created on: Oct 5, 2016
 *      Author: rebeccagraber
 */

#include <cv.h>
#include <highgui.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/core.hpp>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <glob.h>


#ifndef BOUNDARY_HPP_
#define BOUNDARY_HPP_

void myGetBoundaries(cv::Mat&, std::vector<std::vector<cv::Point> >&);

#endif /* BOUNDARY_HPP_ */
