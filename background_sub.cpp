
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

#include "libContour.hpp"
#include "floodfill.hpp"
#include "segmentation.hpp"
using namespace cv;
using namespace std;

Point getLowestPoint(Vector<Point> in)
{
	Point p = Point(0,0);
	bool first;
	for(int i=0; i < in.size(); i++)
	{
//		cout << "P: " << in[i].x << "," << in[i].y;
		if (first || in[i].y > p.y )
		{
			p = in[i];
			first = false;
		}
	}
	return p;
}

Point getLeftMostPoint(Vector<Point> in)
{
	Point p = Point(0,0);
		bool first;
		for(int i=0; i < in.size(); i++)
		{
//			cout << "P: " << in[i].x << "," << in[i].y;
			if (first || in[i].x < p.x )
			{
				p = in[i];
				first = false;
			}
		}
		return p;
}


Point getHighestPoint(Vector<Point> in)
{
	Point p = Point(0,0);
	bool first;
	for(int i=0; i < in.size(); i++)
	{
//		cout << "P: " << in[i].x << "," << in[i].y;
		if (first || in[i].y < p.y )
		{
			p = in[i];
			first = false;
		}
	}
	return p;
}

Point getRightMostPoint(Vector<Point> in)
{
	Point p = Point(0,0);
		bool first;
		for(int i=0; i < in.size(); i++)
		{
//			cout << "P: " << in[i].x << "," << in[i].y;
			if (first || in[i].x > p.x )
			{
				p = in[i];
				first = false;
			}
		}
		return p;
}


Rect rectFromMask(Mat& mask) {
    int l = -1;
    int r = -1;
    int t = -1;
    int b = -1;
    int row, col;
    for ( row = 0; row < mask.rows; ++row ) {
        for ( col = 0; col < mask.cols; ++col ) {
            if (mask.at<uchar>(row, col) == 255) {
                if ( l == -1 ) { l = col; }
                if ( col > r ) { r = col; }
                if ( t == -1 ) { t = row; }
                if ( row > b ) { b = row; }
            }
        }
    }
    int w = r - l;
    int h = b - t;
    Rect roi(l, t, w, h);
    return roi;
}

void mergeHistory(vector<Mat> history, Mat& dst) {
    int rows = history[0].rows;
    int cols = history[0].cols;
    vector<uchar> merging;
    int i, row, col;
    for ( row = 0; row < rows; ++row ) {
        for ( col = 0; col < cols; ++col ) {
            merging.clear();
            for ( i = 0; i < history.size(); ++i ) {
                merging.push_back(history[i].at<uchar>(row, col));
            }
            // take the median intensity across all history frames for
            // this single pixel
            size_t n = merging.size() / 2;
            nth_element(merging.begin(), merging.begin()+n, merging.end());
            int median = merging[n];
            dst.at<uchar>(row, col) = median;

            // alternatively, take the mean
            //double avg = mean(merging)[0];
        }
    }
}

vector<string> filesInFolder(string& dirpath)
{
	dirpath.append("*");
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

// Create a mask of an area identified as matching the input template
void createMatchedMask(string img_file, string template_file, Mat& mask) {
    // load first image to template match
    Mat img = imread(img_file, IMREAD_GRAYSCALE);
    if ( !img.data ) {
        cout << "Could not open file " << img_file << endl;
        return;
    }
    Mat templ = imread(template_file, IMREAD_GRAYSCALE);
    if ( !templ.data ) {
        cout << "Could not open file " << template_file << endl;
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
	string dirpath = argv[1];//"/Users/rebeccagraber/Documents/IVC/HW4/eelB/*";
	vector<string> img_files = filesInFolder(dirpath);

	if ( img_files.size() == 0 ) {
			cout << "Data files in " << dirpath << " were not found!" <<  endl;
			return -1;
	}

    Mat tank_mask;
    createMatchedMask(img_files[0], argv[2],
                        tank_mask);
    if ( tank_mask.empty() ) {
        cout << "ERROR: Failed to create tank mask" << endl;
        return -1;
    }
    Rect tank = rectFromMask(tank_mask);

    vector<Mat> history; // stores the previous N frames

    Mat buffer; // stores the result of intermediate computations
    Mat orig, grey, mergedhist, prev; // orig and helper images
    Mat ffbuffer, ffoutput;
    Mat output, motion, colorized, segmented, sticky_prev, sticky; // results
    Mat flooded_colorized, flooded_contours, flooded_sticky, flooded_sticky_prev; // results with flooding
    Mat display; // final output that is displayed

    map<int, vector<Point> > eels;

    // morphology operator used in an attempt to remove noise in segmentations
    Mat eroded, dilated;
    Mat sElem = getStructuringElement(MORPH_RECT,Size(5,5),Point(0,0));

    // process all video frames
	for ( int i = 0; i < img_files.size(); ++i ) {
		buffer = imread(img_files[i],CV_LOAD_IMAGE_COLOR);
		if(!buffer.data )
		{
			cout << "No image data" <<  endl;
			return -1;
		}

        // isolate tank area
        orig = buffer(tank);

        // create grayscale image
		cvtColor(orig, grey, CV_BGR2GRAY);
		blur(grey, buffer, Size(3,3));
		grey = buffer.clone();

        if ( !mergedhist.empty() ) {
            // *******************************************
            // **** Segment via motion detection ****
            // get difference between the merged history and current frame
            subtract(mergedhist, grey, output);
            threshold(output, output, 10, 255, 0);

            // get difference between the previous and current frame
            motion = (prev - grey) > 15;
            dilate(motion,dilated,sElem);
            erode(dilated,eroded,sElem);
            motion = eroded;

            // attempt to identify non-moving eels, which are not detected
            // in the motion detection
            Mat known_still,known_still_prev;
            known_still = Mat::zeros(grey.size(), CV_8U);

            if ( !sticky_prev.empty() ) {
                prev.copyTo(known_still_prev, (sticky_prev > 1));
                grey.copyTo(known_still, (sticky_prev > 1));
                known_still = ((known_still - known_still_prev) > 5);
            }
            imshow("dbg",known_still);
            cout << output.type() << ',' << motion.type() << ',' << known_still.type() << '\n';
            buffer = output+motion;//+known_still;

            // *******************************************
            // *** Associate the same segments across frames ***
            myFindContours(buffer, segmented);

            if ( !sticky_prev.empty() ) {
                make_sticky(sticky_prev, segmented, sticky);
            }
            else {
                sticky=segmented;
            }
            sticky_prev = sticky;
            colorize(sticky, colorized);

            // *******************************************
            // *** Flood fill segments identified as likely eels ***
            // get range of identified segment ID's on which to iterate
            double min, max;
            minMaxLoc(sticky, &min, &max);
            int nmax = (int) max;

        /*    // flood fill each segment to expand to full body of eel
            ffbuffer = Mat::zeros(sticky.size(), CV_32S);
            for ( int i = 1; i < nmax; i++) {
                cout << "Obj_id: " << i << endl;
                floodFillFromSegment(grey, sticky, i, ffbuffer);
            }
            ffbuffer.convertTo(ffoutput, CV_8UC1);
            threshold(ffoutput, ffoutput, 1, 255, 0);*/


            //***** HEAD AND TAIL ******//
            Mat eel_points = Mat::zeros(colorized.rows, colorized.cols, CV_8UC3);
            Mat skeleton, skeleton_conv;
            skeleton = Mat::zeros(sticky.rows, sticky.cols, CV_8U);

            for(int i=1; i< nmax; i++)
            {
            	cout << i << endl;
    //        	cout << type2str(sticky.type()) << endl;

            	vector<Point> skel;
            	skel = skeletonize(sticky,skeleton,i);
            	cout << "skel " << skel.size() << endl;


            	vector<Point> eel;
            	Point cent = centroid(skel);
            	cout << "Found centroid " << endl;
		if (cent.x < 0 || cent.y < 0)
			continue;
            	Point old_cent = cent;
            	Point head;
            	Point tail;
            	if(eels.count(i) != 0)
            	{
            		old_cent = eels[i][0];
            	}

            	double theta = orientation(skel);
            	cout << "Found orientation " << endl;
            	 if (theta < 2.35 && theta > 0.785 ) {
            		 //vertical
            		 if(old_cent.y < cent.y) //moving downwards
            		 {
            			 head = getLowestPoint(skel);
            			 tail = getHighestPoint(skel);
            		 }
            		 else //moving upwards
            		 {
            			 head = getHighestPoint(skel);
            			 tail = getLowestPoint(skel);
            		 }
            	 }
            	 else //horizontal
            	 {
            		 if(old_cent.x < cent.x) //moving right
            		 {
            		    head = getRightMostPoint(skel);
            		    tail = getLeftMostPoint(skel);
            		  }
            		  else //moving upwards
            		  {
            		    head = getLeftMostPoint(skel);
            		    tail = getRightMostPoint(skel);
            		  }
            	 }
         //   	 cout << "Found head and tail" << endl;
            	 eel.push_back(cent);
            	 eel.push_back(head);
            	 eel.push_back(tail);
            	 eels[i] = eel;
         //   	 cout << "Added to map" << endl;
            	 cout << "Head " << head.x << "," << head.y << endl;
            	 eel_points.at<Vec3b>(head.y,head.x) = Vec3b(0,0,255);
            	 cout << "Centroid " << cent.x << "," << cent.y << endl;
            	 eel_points.at<Vec3b>(tail.y,tail.x) = Vec3b(0,255,0);
            	 eel_points.at<Vec3b>(cent.y,cent.x) = Vec3b(255,0,0);
            	 circle(eel_points, tail, 10, Scalar(0,255,0));
            	 circle(eel_points, head, 10, Scalar(0,0,255));
            	 circle(eel_points, cent, 10, Scalar(255,0,0));
cout << "eel_points" << endl;

            }



            // ******* COMBINE FLOOD FILL OUTPUT AND STICKY OUTPUT HERE
            // !!! AND uncomment to display the result!

/*
            myFindContours(ffoutput,flooded_contours);
            if (!flooded_sticky_prev.empty() ) {
                   make_sticky(flooded_sticky_prev, flooded_contours, flooded_sticky);
                }
                else {
                   flooded_sticky_prev=flooded_contours;
                }
             flooded_sticky_prev = flooded_sticky;
             colorize(flooded_sticky, flooded_colorized);
*/



            // *******************************************
            // *** Create output frame and display ***
            // create output view to show segmentation and original side by side
            display = Mat(orig.rows*2, orig.cols, orig.type());

            // copy original grey image to output frame
            cvtColor(grey, buffer, CV_GRAY2BGR);
            cvtColor(skeleton,skeleton_conv,CV_GRAY2BGR);
            buffer.copyTo(display(Rect(0, 0, buffer.cols, buffer.rows)));

            // copy final segmentation to output frame
            buffer = colorized + eel_points + skeleton_conv;

            buffer.copyTo(display(Rect(0, buffer.rows, buffer.cols, buffer.rows)));

            // show output frame
            imshow("Output", display);

            // If ESC key pressed, exit program
            if (waitKey(30) == 27) {
                break;
            }
        }

        // use current frame as previous frame in next iteration
        prev = grey.clone();

        // update the history, and merge if enough frames have passed
        history.push_back(grey.clone());
        if ( history.size() > 10 ) {
            // cycle history by deleting oldest entry
            history.erase(history.begin());

            // merge history into a single Mat
            mergedhist = Mat::zeros(grey.size(), grey.type());
            mergeHistory(history, mergedhist);
        }

	}
	waitKey(0);
    return 0;
}
