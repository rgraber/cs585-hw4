#include "utils.hpp"

//assumes a 10FPS rate, idx 0 == 00:00:00
string frameNumberToTime(int idx, int fps) {
    double total_seconds = (double)idx / (double)fps;

    int hours = total_seconds / (60*60);
    total_seconds = total_seconds - (hours * 60*60);
    int mins = total_seconds / (60);
    total_seconds = total_seconds - (mins * 60);
    double secs = total_seconds;

    string delim = ":";
    string timestamp = to_string(hours) + delim + to_string(mins)
                                                    + delim + to_string(secs);
    return timestamp;
}

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

