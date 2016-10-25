#include "hw4.hpp"

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

/* Returns a vector of locations with a pixel intensity equal
 * to the input obj_id */
vector<Point> foreground(Mat& src, int obj_id) {
    vector<Point> foreground;
    int row, col;
    for ( row = 0; row < src.rows; ++row ) {
        for ( col = 0; col < src.cols; ++col ) {
            if (src.at<uchar>(row, col) == obj_id) {
                Point p = Point(col, row);
                foreground.push_back(p);
            }
        }
    }
    return foreground;
}

/* Normalize values in vector between a and b. In-place. */
void normalize(vector<double>& vec, double a, double b) {
    double minv, maxv;
    minMaxLoc(vec, &minv, &maxv);
    for ( int i = 0; i < vec.size(); ++i ) {
        double normalized = a + (((vec[i] - minv) * (b - a))  / (maxv - minv));
        vec[i] = normalized;
    }
}

vector<Point> drawPointsOnLine(Mat& src, Point center, double theta) {
    theta = -theta;
    // center: centroid
    // theta: orientation angle in radians
    double g = -center.x*sin(theta) + center.y*cos(theta);

    double x1 = 0;
    double x2 = src.cols;
    double y1 = ((x1*sin(theta)) + g) / (cos(theta));
    double y2 = ((x2*sin(theta)) + g) / (cos(theta));

    Point p1 = Point(x1, y1);
    Point p2 = Point(x2, y2);

    vector<Point> points;
    points.push_back(p1);
    points.push_back(p2);

    line(src, points[0], points[1], 255);

    return points;
}
