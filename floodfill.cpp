
#include "floodfill.hpp"

void floodFillInner(Mat& grey, Mat& result, int obj_id, Point p, int nmean, Mat& segmented)
{
	if(p.y > grey.rows || p.x > grey.cols)
	{
		cout << "out of bounds: " << grey.rows << "," << grey.cols << endl;
		return;
	}
	if(result.at<int>(p.y,p.x) == obj_id)
	{
		return;
	}
	int val = (int)grey.at<uchar>(p.y,p.x);
	if (((val < nmean + 1) && (val > nmean - 1)) || segmented.at<int>(p.y,p.x) == obj_id)
	{
		result.at<int>(p.y,p.x) = obj_id;
		Point p0 = Point(p.x+1,p.y);
		Point p1 = Point(p.x, p.y+1);
		Point p2 = Point(p.x, p.y-1);
		Point p3 = Point(p.x-1, p.y);
		floodFillInner(grey,result,obj_id,p0,nmean,segmented);
		floodFillInner(grey,result,obj_id,p1,nmean,segmented);
		floodFillInner(grey,result,obj_id,p2,nmean,segmented);
		floodFillInner(grey,result,obj_id,p3,nmean,segmented);
	}

}


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

	floodFillInner(grey, result, obj_id, p, nmean,segments);

	}
}
