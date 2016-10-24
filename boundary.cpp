/*
 * boundary.cpp
 *
 *  Created on: Oct 4, 2016
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

#include "boundary.hpp"

using namespace cv;
using namespace std;

void myGetBoundaries(Mat& segmented, vector<vector<Point> > &out)
{
	double min_f, max_f;

	minMaxLoc(segmented, &min_f, &max_f);

	int max_i = int(max_f);
	for(int seg = 1; seg < max_i; seg++)
	{

		vector<Point> bound;
		Point s;
		bool found = false;
		//scan image to find starting point
		for(int i = 0; i < segmented.rows; i++)
		{
			for(int j = 0; j < segmented.cols; j++)
			{
				if(segmented.at<int>(i,j) == seg)
				{
					s = Point(j,i);
					found = true;
					break;
				}
			}
			if(found) break;
		}
		if(!found)
		{
			continue;
		}

		Point c = Point(s.x,s.y);
		Point b = Point(c.x - 1, c.y);
		bool first = true;
		vector<Point> v(8);
		while(first || !(s.x == c.x && s.y == c.y))
		{
		//	cout << "C: " << c.x << "," << c.y << endl;
		//	cout << "B: " << b.x << "," << b.y << endl;
			bound.push_back(c);
		//	segmented.at<int>(c.y,c.x) = 255;
			first = false;
			v.clear();
			//N8, clockwise
			v.push_back(Point(c.x - 1, c.y));
			v.push_back(Point(c.x - 1, c.y + 1));
			v.push_back(Point(c.x, c.y + 1));
			v.push_back(Point(c.x + 1, c.y + 1));
			v.push_back(Point(c.x + 1, c.y));
			v.push_back(Point(c.x + 1, c.y - 1));
			v.push_back(Point(c.x, c.y - 1));
			v.push_back(Point(c.x - 1, c.y - 1));

			int index = 0;
			for(int i = 0; i < 8; i++)
			{
				//find b
				index = i;
				Point a = v.at(i);
				if(a.x == b.x && a.y == b.y)
				{
					break;
				}
			}

			//	cout << "Index of B: " << index << endl;

			//going clockwise from b, find next element in object
			Point next_b = Point(b.x,b.y);
			int now = (index + 1) % 8;
			Point p;
			while(now != index)
			{
				//	cout << "Now: " << now << endl;
				//is this point in the object?
				p = v.at(now);
				//	cout << "P Now: " << p.x << "," << p.y << endl;
				if(segmented.at<int>(p.y,p.x) == seg)
				{
					//we found the new c!
					c = Point(p.x,p.y);
					b = Point(next_b.x,next_b.y);
					break;
				}
				else
				{
					next_b = Point(p.x,p.y);
					now = (now + 1) % 8;
				}
			}
		}
		//cout << "bound size " << bound.size() << endl;
		if(bound.size() > 1)
		{
			out.push_back(bound);
		}
		//cout << "out size " << out.size() << endl;
	}
}

