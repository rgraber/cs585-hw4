
#include "floodfill.hpp"

void floodFillInner(Mat& grey, Mat& result, int obj_id, Point p, int nmean, int dev, Mat& segmented, int count)
{

	cout << "Testing " << p.x << "," << p.y << endl;
	cout << "object id " << obj_id << endl;
	int s = segmented.at<int>(p.y,p.x);
	cout << "segment value at point " << s << endl;
	count++;

	if(p.y > grey.rows || p.x > grey.cols)
	{
		cout << "out of bounds: " << grey.rows << "," << grey.cols << endl;
		return;
	}
	if(result.at<int>(p.y,p.x) == obj_id)
	{
//		cout << "Already marked" << endl;
		return;
	}
	int val = (int)grey.at<uchar>(p.y,p.x);
	cout << "Grey val: " << val << endl;
//	cout << "Seg val: " << segmented.at<int>(p.y,p.x) << endl;
	if (((val < nmean + 1) && (val > nmean - 1)) || segmented.at<int>(p.y,p.x) == obj_id)
	{
		result.at<int>(p.y,p.x) = obj_id;
		Point p0 = Point(p.x+1,p.y);
		Point p1 = Point(p.x, p.y+1);
		Point p2 = Point(p.x, p.y-1);
		Point p3 = Point(p.x-1, p.y);
		floodFillInner(grey,result,obj_id,p0,nmean,dev,segmented,count);
		floodFillInner(grey,result,obj_id,p1,nmean,dev,segmented,count);
		floodFillInner(grey,result,obj_id,p2,nmean,dev,segmented,count);
		floodFillInner(grey,result,obj_id,p3,nmean,dev,segmented,count);
	}

}


//orig is grey here
void floodFillFromSegment(Mat &grey, Mat &segments, int obj_id, Mat &result)
{
	//find point on object
	Point p;
	bool found = false;
	int sum=0;
	int count=0;

	//get mean brightness of segmented object
	Mat mask = Mat::zeros(grey.rows, grey.cols, CV_8U);
	for(int i=0; i < grey.rows; i++)
	{
		for(int j=0; j< grey.cols; j++)
		{
			if(segments.at<int>(i,j) == obj_id)
			{
				sum+=(int)(grey.at<uchar>(i,j));
				count++;
			}
		}
	}

	int nmean = sum/count;
	int dev = 0;
	//get standard dev, in stupid way
	for(int i=0; i < grey.rows; i++)
		{
			for(int j=0; j< grey.cols; j++)
			{
				if(segments.at<int>(i,j) == obj_id)
				{
					uchar v = grey.at<uchar>(i,j);
					int nv = (int)v;
					dev+=(nv - nmean)*(nv - nmean);
				}
			}
		}
	dev = sqrt(dev/count);
	cout << "Mean: " << nmean << endl;
	cout << "Std dev: " << dev << endl;

	for(int i=0; i < segments.rows; i++)
	{
		for(int j=0; j < segments.cols; j++)
		{
			if (segments.at<int>(i,j) == obj_id)
			{

				p = Point(j,i);
				found = true;
				break;
			}
			if(found) break;
		}
	}
	if(!found) return;
	else
	{
	cout << "Starting at point " << p.x << "," << p.y << endl;
	cout << "At start " << segments.at<int>(p.y,p.x) << endl;
	if(count > 30)
	{
	floodFillInner(grey, result, obj_id, p, nmean, dev, segments,0);
	}
	}
}
