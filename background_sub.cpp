/*
 * background_sub.cpp
 *
 *  Created on: Oct 15, 2016
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

using namespace cv;
using namespace std;

vector<string> filesInFolder(string& dirpath)
{
	glob_t glob_result;
	glob( dirpath.c_str(), GLOB_TILDE, NULL, &glob_result );
	vector<string> files;
	for ( int i = 0; i < glob_result.gl_pathc; ++i ) {
		files.push_back(string(glob_result.gl_pathv[i]));
	}
	globfree(&glob_result);
	return files;
}

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void createMatchedMask(string img_file, string template_file, Mat& mask) {
    // load first image to template match
    Mat img = imread(img_file, IMREAD_GRAYSCALE);
    Mat templ = imread(template_file, IMREAD_GRAYSCALE);
    if ( !img.data || !templ.data ) {
        cout << "Could not open file " << img_file << " or " <<
                                          template_file << endl;
        return;
    }

    int match_method = CV_TM_SQDIFF_NORMED;
    int result_cols = img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    Mat result;
    result.create( result_rows, result_cols, CV_32FC1 );
    matchTemplate( img, templ, result, match_method );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

	/// Localizing the best match with minMaxLoc
	double minVal; double maxVal; Point minLoc; Point maxLoc;
	Point matchLoc;

	minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

	if( match_method  == CV_TM_SQDIFF ||
            match_method == CV_TM_SQDIFF_NORMED ){
		matchLoc = minLoc;
    } else {
        matchLoc = maxLoc;
    }

    mask = Mat::zeros(img.size(), img.type());
    rectangle( mask, matchLoc,
            Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ),
            Scalar::all(255), -1);
}

int main(int argc, char** argv)
{

	Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor

	pMOG2 = new BackgroundSubtractorMOG2(100,10,false);
	string dirpath = "data/eels/*";
	vector<string> img_files = filesInFolder(dirpath);
	namedWindow("window",WINDOW_AUTOSIZE);
	namedWindow("final",WINDOW_AUTOSIZE);
	namedWindow("grey",WINDOW_AUTOSIZE);

	if ( img_files.size() == 0 ) {
			cout << "Data files in " << dirpath << " were not found!" <<  endl;
			return -1;
	}

    Mat tank_mask;
    createMatchedMask( img_files[0],
                       "data/background_template.png", tank_mask);
    if ( tank_mask.empty() ) {
        cout << "ERROR: Failed to create tank mask" << endl;
        return -1;
    }
    resize(tank_mask, tank_mask, Size(), 0.5, 0.5, INTER_LANCZOS4);

	Mat orig, smaller, mask_flip, mask_color, mask_convert, grey, thresh1, thresh2;
	Mat thresh1_color,thresh2_color, thresh1_convert, thresh2_convert, fgMaskMOG2, sElem, eroded, dilated;
	Mat segmented, old, diff, grey1;
	bool f = false;
	for ( int i = 0; i < img_files.size(); ++i ) {

		orig = imread(img_files[i],CV_LOAD_IMAGE_COLOR);
		if(!orig.data )
		{
			cout << "No image data" <<  endl;
			return -1;
		}

		resize(orig, orig, Size(), 0.5, 0.5,INTER_LANCZOS4);
        orig.copyTo(smaller, tank_mask);
		if(!f)
		{
			old = smaller;
			f = true;
			continue;
		}
		cvtColor(smaller,grey1,CV_BGR2GRAY);
		Scalar a = mean(grey1);

		grey = grey1 - a;


		imshow("grey",grey);
		Mat equal;
		//equalizeHist(grey,equal);
		threshold(grey,thresh1,255, 255, THRESH_BINARY);
		threshold(grey,thresh2,0, 255, THRESH_BINARY_INV);
		cvtColor(thresh1, thresh1_color,CV_GRAY2BGR );
		cvtColor(thresh2, thresh2_color,CV_GRAY2BGR );
		thresh1_color.convertTo(thresh1_convert, CV_8UC3);
		thresh2_color.convertTo(thresh2_convert, CV_8UC3);



		pMOG2->operator()(grey, fgMaskMOG2, .0025);
		bitwise_not(fgMaskMOG2, mask_flip);
		cvtColor(mask_flip, mask_color, CV_GRAY2BGR);

		mask_color.convertTo(mask_convert, CV_8UC3);

		segmented = ((smaller - thresh1_convert) - thresh2_convert) - mask_convert;
		old = smaller;

		imshow("window",smaller);


		sElem = getStructuringElement(MORPH_RECT,Size(3,3),Point(0,0));
		erode(fgMaskMOG2,eroded,sElem);
		dilate(eroded,dilated,sElem);
		imshow("final",dilated);

		if (waitKey(30) == 27) {
			break;
		}


	}
	waitKey(0);



	    return 0;
}
