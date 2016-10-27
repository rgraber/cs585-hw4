
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

#include "utils.hpp"
#include "libContour.hpp"
#include "floodfill.hpp"
#include "segmentation.hpp"
using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
    // **************************************
    // Open activity record file for recording data to CSV
    ofstream analysis_record;
    analysis_record.open("analysis_record.csv");
    ofstream activity_record;
    activity_record.open("activity_record.csv");
    // ***************************************
    bool activity_found = false;
    int last_activity_found = -1;
    int start_of_activity = -1;

	string dirpath = "data/eels2/*";
	vector<string> img_files = filesInFolder(dirpath);

	if ( img_files.size() == 0 ) {
			cout << "Data files in " << dirpath << " were not found!" <<  endl;
			return -1;
	}

    Mat tank_mask;
    createMatchedMask(img_files[0], argv[2], tank_mask);
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
            for ( int j = 1; j < nmax; j++) {
                cout << "Obj_id: " << j << endl;
                floodFillFromSegment(grey, sticky, j, ffbuffer);
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

            // *** determine if activity was found ***
            if ( sum(buffer)[0] > 0 ) {
                cout << sum(buffer) << endl;
                last_activity_found = i;
                if ( !activity_found ) {
                    start_of_activity = i;
                    activity_found = true;
                }
            }

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

        // record start/end of activity period if applicable
        if ( activity_found && ( i - last_activity_found ) > 10 ) {
            activity_record << frameNumberToTime(start_of_activity);
            activity_record << " to ";
            activity_record << frameNumberToTime(last_activity_found);
            activity_record << "\n";

            activity_found = false;
        }

        // save activity data to a file
        analysis_record << to_string(i) << ", ";
        analysis_record << frameNumberToTime(i);// << ",";
        analysis_record << "\n";
	}

    // final write out
    if ( activity_found ) {
        activity_record << frameNumberToTime(start_of_activity);
        activity_record << " to ";
        activity_record << frameNumberToTime(last_activity_found);
        activity_record << "\n";

        activity_found = false;
    }

    // close output file
    activity_record.close();
    analysis_record.close();

	waitKey(0);
    return 0;
}
