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

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
	string dirpath = "data/eels2/*";
	vector<string> img_files = filesInFolder(dirpath);

	if ( img_files.size() == 0 ) {
			cout << "Data files in " << dirpath << " were not found!" <<  endl;
			return -1;
	}

    cout << "TOTAL TIME" << endl;
    cout << frameNumberToTime(img_files.size(), 10) << endl;
    return -1;

    Mat tank_mask;
    createMatchedMask(img_files[0], "data/background_template.png",
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
    Mat display; // final output that is displayed

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
            threshold(output, output, 20, 255, 0);

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
            buffer = output+motion+known_still;

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

            // flood fill each segment to expand to full body of eel
            ffbuffer = Mat::zeros(sticky.size(), CV_32S);
            for ( int i = 1; i < nmax; i++) {
                cout << "Obj_id: " << i << endl;
                floodFillFromSegment(grey, sticky, i, ffbuffer);
            }

            ffbuffer.convertTo(ffoutput, CV_8UC1);
            threshold(ffoutput, ffoutput, 1, 255, 0);


            // ******* COMBINE FLOOD FILL OUTPUT AND STICKY OUTPUT HERE
            // !!! AND uncomment to display the result!


            // *******************************************
            // *** Create output frame and display ***
            // create output view to show segmentation and original side by side
            display = Mat(orig.rows*2, orig.cols, orig.type());

            // copy original grey image to output frame
            cvtColor(grey, buffer, CV_GRAY2BGR);
            buffer.copyTo(display(Rect(0, 0, buffer.cols, buffer.rows)));

            // copy final segmentation to output frame
            buffer = colorized;
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
