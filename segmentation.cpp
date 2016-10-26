#include "segmentation.hpp"

vector<Point> skeletonToContour(Mat& src, int obj_id) {
    // The starting point could be selected more intelligently to ensure that
    // it is not at the edge of a curve.
    // Currently expected to be in the center of a non-curve, like so:
    //      -------s---...
    //    /      <-|->
    //   /       <-|->_...
    //   |     /
    //   |     \
    //   \       ------...
    //    \
    //      -----------...
    //
    // Overall, the algorithm works in the following way:
    // 1. Finds a random point on the border of the object
    // 2. Creates a dividing line through the segment of the object attached
    //    to the found point. If the orientation is vertical, it sweeps
    //    to the right. If the orientation is horizontal, it sweeps down.
    // 3. Separates the neighbors of the divide line into "left" or "right"
    //    search spaces. These are misnomers, b/c they could be the "top"
    //    and "bottom" search spaces if working with a vertically orientated
    //    object.
    // 4. With the separated search spaces, the algorithm proceeds to flood
    //    fill the sides separately by finding all N8 neighbors connected
    //    to the current search space. On each iteration, the vector point
    //    added to the returned contour is calculated as the centroid of the
    //    current search space.

    // Mat used to mark which pixels in the object have already been visited
    Mat buffer = Mat::zeros(src.size(), CV_32S);

    // Find a single point on the border of the object
    Point s = borderPoint(src, obj_id);
 //   cout << "Border pt "<< s.x << "," << s.y << endl;

    int found = obj_id;
    vector<Point> dividingLine;
    vector<Point> searchSpace;
    vector<Point> leftSearch;
    vector<Point> rightSearch;

    Point p;
    double theta = orientation(src, obj_id);
    if (theta < 2.35 && theta > 0.785 ) {
//    cout << "vertical" << endl;
        // Assumes a vertical orientation

        // Find the dividing line from the start point
        for ( int col = s.x; col < src.cols - s.x; ++col ) {
            if ( src.at<int>(s.y, col) == obj_id ) {

                p = Point(col, s.y);
//                cout << "Dividing line p " << p.x << "," << p.y << endl;
                dividingLine.push_back(p);
                searchSpace.push_back(p);
            } else {
                break;
            }
        }

        // Create the initial search space
        neighborhood(dividingLine, searchSpace);

        // Separate into top ("right") and bottom ("left") search spaces
        for (auto v = searchSpace.begin(); v != searchSpace.end(); ++v ) {
            p = *v;
            buffer.at<int>(p.y, p.x) = found;
            if ( p.y < s.y ) {
                leftSearch.push_back(p);
            } else if ( p.y > s.y ) {
                rightSearch.push_back(p);
            }
        }

    } else {
//    	cout << "horiz" << endl;
//    	cout << "Base " << (s.y) << endl;
//    	cout << "End " << (src.rows - s.y) << endl;
//    	cout << "X " << s.x << endl;
        // Assumes a horizontal orientation
//s.y+1
        // Find the dividing line from the start point (2)
        for ( int row = s.y ; row < src.rows; ++row ) {
//        	cout << "Row " << row << endl;
//        	cout << "id " << src.at<int>(row,s.x) << endl;
            if ( src.at<int>(row, s.x) == obj_id ) {
                p = Point(s.x, row);
 //               cout << "Dividing line p " << p.x << "," << p.y << endl;
                dividingLine.push_back(p);
                searchSpace.push_back(p);
            } else {
                break;
            }
        }

        // Create the initial search space
        neighborhood(dividingLine, searchSpace);

        // Separate into left and right search spaces
        for (auto v = searchSpace.begin(); v != searchSpace.end(); ++v ) {
            p = *v;
            buffer.at<int>(p.y, p.x) = found;
            if ( p.x < s.x ) {
                leftSearch.push_back(p);
            } else if ( p.x > s.x ) {
                rightSearch.push_back(p);
            }
        }
    }


    // Create separate left and right vectors
    vector<Point> leftVectorized = floodFillAndVectorize(
                                            src, buffer, leftSearch, found);
    vector<Point> rightVectorized = floodFillAndVectorize(
                                            src, buffer, rightSearch, found);

    // Add initial point to end of left vector to maintain order
    rightVectorized.push_back(centroid(dividingLine));

    // Merge vectors in order
    vector<Point> merged;
    merged.reserve(leftVectorized.size() + rightVectorized.size() );
    reverse(rightVectorized.begin(), rightVectorized.end());
    merged.insert(merged.end(), rightVectorized.begin(), rightVectorized.end());
    merged.insert(merged.end(), leftVectorized.begin(), leftVectorized.end());

    return merged;
}

vector<Point> floodFillAndVectorize(Mat& src, Mat& buffer,
                                    vector<Point> searchSpace, int found) {

    vector<Point> vectorized;

    int i;
    vector<Point> neighbors;
    while ( searchSpace.size() > 0 ) {
        // calculate the center point of the new search space to be used
        // as a point in the final vector of the shape
        vectorized.push_back(centroid(searchSpace));

        // find neighborhood for the current search space
        neighbors.clear();
        neighborhood(searchSpace, neighbors);

        // find new search space based on neighbors that are a part of object
        searchSpace.clear();
        Point p;
        for ( i = 0; i < neighbors.size(); ++i ) {
            p = neighbors[i];
            if ( inBounds(p, src.cols, src.rows) ) {
                if ( src.at<int>(p.y, p.x) == found &&
                            buffer.at<int>(p.y, p.x) != found ) {
                    buffer.at<int>(p.y, p.x) = found;
                    searchSpace.push_back(p);
                }
            }
        }
    }

    return vectorized;
}

//http://stackoverflow.com/a/20947961
struct comparePoints {
    bool operator()(const Point & a, const Point & b) {
        return (a.x != b.x || a.y != b.y);
    }
};

void neighborhood(vector<Point> points, vector<Point>& neighbors) {
    set<Point, comparePoints> uNeighbors; // unique neighbors
    Point p;
    for ( int i = 0; i < points.size(); ++i ) {
        p = points[i];
        // above/below
        uNeighbors.insert(Point(p.x, p.y-1));
        uNeighbors.insert(Point(p.x, p.y+1));
        // right neighborhood
        uNeighbors.insert(Point(p.x-1, p.y-1));
        uNeighbors.insert(Point(p.x-1, p.y));
        uNeighbors.insert(Point(p.x-1, p.y+1));
        // left neighborhood
        uNeighbors.insert(Point(p.x+1, p.y+1));
        uNeighbors.insert(Point(p.x+1, p.y));
        uNeighbors.insert(Point(p.x+1, p.y-1));
    }
    set<Point,comparePoints>::iterator it;
    for ( it = uNeighbors.begin(); it != uNeighbors.end(); ++it ) {
        neighbors.push_back(*it);
    }
}


bool inBounds(Point p, int w, int h) {
    return (p.x >= 0 && p.x < w && p.y >= 0 && p.y < h);
}

/* Returns the skeleton of the binary input image src, drawn on
 * the output Mat object, dst. Assumes the foreground to be marked
 * in white and the background black.  */
vector<Point> skeletonize(Mat& src, Mat& dst,   int obj_id) {
	cout << "Obj id " << obj_id << endl;
    Mat buffer = Mat::zeros(src.size(), CV_32S);
    distanceToBackground(src, buffer, obj_id);
    vector<Point> out;

    int row, col;
    int dist;
    for ( row = 0; row < src.rows; ++row ) {
   // 	cout << "row " << row << endl;


        for ( col = 0; col < src.cols; ++col ) {
        	int val = src.at<int>(row,col);
        	//cout << "val " << val << endl;
            if ( src.at<int>(row, col) == obj_id ) {
                dist = buffer.at<int>(row, col);
                cout << "dist " << dist << endl;
                if (buffer.at<int>(row-1, col) <= dist &&
                    buffer.at<int>(row+1, col) <= dist &&
                    buffer.at<int>(row, col-1) <= dist &&
                    buffer.at<int>(row, col+1) <= dist) {
                    dst.at<uchar>(row, col) = 255;
                    cout << "pushing back" << endl;
                    out.push_back(Point(col,row));
                }
            }
        }
    }
    return out;
}

/* Creates a distance Mat, which marks the distance of every pixel with
 * intensity == obj_id in src to the background marked with 0 intensities.  */
void distanceToBackground(Mat& src, Mat& dst, int obj_id) {
    vector<Point> object = foreground(src, obj_id);
    Point p;
    for ( int i = 0; i < object.size(); ++i ) {
        p = object[i];
        // walk across n4 neighborhood to find closest background pixel
        int dist = 0;
        bool continueSearch = true;
        while ( continueSearch ) {
            ++dist;
            continueSearch = false;
            // top
            if ( p.y - dist >= 0 ) {
                if ( src.at<int>(p.y - dist, p.x) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // bottom
            if ( p.y + dist < src.rows ) {
                if ( src.at<int>(p.y + dist, p.x) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // left
            if ( p.x - dist >= 0 ) {
                if ( src.at<int>(p.y, p.x - dist) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // right
            if ( p.x + dist < src.cols ) {
                if ( src.at<int>(p.y, p.x + dist) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            /*
            // diagonal top-right
            if ( p.x + dist < src.cols && p.y - dist >= 0 ) {
                if ( src.at<uchar>(p.y - dist, p.x + dist) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // diagonal bottom-right
            if ( p.x + dist < src.cols && p.y + dist < src.rows ) {
                if ( src.at<uchar>(p.y + dist, p.x + dist) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // diagonal top-left
            if ( p.x - dist >= 0 && p.y - dist >= 0 ) {
                if ( src.at<uchar>(p.y - dist, p.x - dist) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // diagonal bottom-left
            if ( p.x - dist >= 0 && p.y + dist < src.rows ) {
                if ( src.at<uchar>(p.y - dist, p.x + dist) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            */
        }
        // assign dist to value
        dst.at<int>(p.y, p.x) =  dist;
    }
}

Point borderPoint(Mat& src, int obj_id) {
    Point p;
    int row, col;
    for ( row = 0; row < src.rows; ++row ) {
        for ( col = 0; col < src.cols; ++col ) {
            if (src.at<int>(row, col) == obj_id) {
                p = Point(col, row);
                return p;
            }
        }
    }
    return p;
}

/*vector<string> filesInFolder(string& dirpath)
{
    glob_t glob_result;
    glob( dirpath.c_str(), GLOB_TILDE, NULL, &glob_result );
    vector<string> files;
    for ( int i = 0; i < glob_result.gl_pathc; ++i ) {
        files.push_back(string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return files;
}*/

/* Returns a vector of locations with a pixel intensity equal
 * to the input obj_id */
vector<Point> foreground(Mat& src, int obj_id) {
    vector<Point> foreground;
    int row, col;
    for ( row = 0; row < src.rows; ++row ) {
        for ( col = 0; col < src.cols; ++col ) {
            if (src.at<int>(row, col) == obj_id) {
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

double circularity(Mat& src, int obj_id) {
    vector<Point> object = foreground(src, obj_id);
    return circularity(object);
}

double circularity(vector<Point> object) {
    Point center = centroid(object);
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    Point p;
    for ( int i = 0; i < object.size(); ++i ) {
        p = object[i];
        a += pow((p.x - center.x), 2);
        b += (p.x - center.x) * (p.y - center.y);
        c += pow((p.y - center.y), 2);
    }
    b = 2 * b;
    double circularity = -1;
    if ( b != 0 && a != c ) {
        double h = sqrt(((a-c)*(a-c)) + (b*b));
        double emin = ((a+c)/2) - (((a-c)/2)*((a-c)/h)) - ((b/2)*(b/h));
        double emax = ((a+c)/2) + (((a-c)/2)*((a-c)/h)) + ((b/2)*(b/h));
        circularity = emin / emax;
    }
    return circularity;
}

double orientation(Mat& src, int obj_id) {
    vector<Point> object = foreground(src, obj_id);
    return orientation(object);
}

double orientation(vector<Point> object) {
    Point center = centroid(object);
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    Point p;
    for ( int i = 0; i < object.size(); ++i ) {
        p = object[i];
        a += pow((p.x - center.x), 2);
        b += (p.x - center.x) * (p.y - center.y);
        c += pow((p.y - center.y), 2);
    }
    //double theta = (b != 0 && a != c) ? (0.5 * atan( 2*b / (a - c) )) : -1;
    double theta = (b != 0 && a != c) ?
                    (0.5 * acos( (a-c) / sqrt(pow(b, 2) + pow(a-c, 2)))) : 0;

    return theta;
}

Point centroid(Mat& src, int obj_id) {
    vector<Point> object = foreground(src, obj_id);
    return centroid(object);
}

Point centroid(vector<Point> object) {
    Point p;
    double xbar = 0.0;
    double ybar = 0.0;
    double area = 0.0;
    for ( int i = 0; i < object.size(); ++i ) {
        p = object[i];
        xbar += p.x;
        ybar += p.y;
        area += 1;
    }
    xbar = xbar / area;
    ybar = ybar / area;
    return Point(xbar, ybar);
}

int area(Mat& src, int obj_id) {
    vector<Point> object = foreground(src, obj_id);
    return area(object);
}

int area(vector<Point> object) {
    return object.size();
}

/* Discrete approximation of (dtheta/ds), where dtheta is the change
 * in angle and ds is the change in arc length.
 * Produces a curvature which ranges from 0 to PI.  */
vector<double> curvature_dtds(vector<Point> contour) {
    vector<double> curvature;

    // adjust contour to handle start and end positions
    contour.insert(contour.begin(), contour[contour.size() - 1]);
    contour.push_back(contour[1]);

    // assign contour calculation for all points in original contour
    for ( int i = 1; i < contour.size() - 1; ++i ) {
        vector<double> uj;
        uj.push_back(contour[i].x - contour[i-1].x);
        uj.push_back(contour[i].y - contour[i-1].y);
        double ujmag = sqrt(pow(uj[0],2) + pow(uj[1],2));

        vector<double> uj1;
        uj1.push_back(contour[i+1].x - contour[i].x);
        uj1.push_back(contour[i+1].y - contour[i].y);
        double uj1mag = sqrt(pow(uj1[0],2) + pow(uj1[1],2));

        double uj_dot_uj1 = uj[0]*uj1[0] + uj[1]*uj1[1];
        uj_dot_uj1 = uj_dot_uj1 / (ujmag * uj1mag);

        double dtheta = acos(uj_dot_uj1);
        double ds = (ujmag + uj1mag) / 2.0;

        double c = dtheta / ds;

        curvature.push_back(c);
    }

    normalize(curvature);

    return curvature;
}

vector<double> curvature_vss(vector<Point> contour) {
    vector<double> curvature;
    // adjust contour to handle start and end positions
    contour.insert(contour.begin(), contour[contour.size() - 1]);
    contour.push_back(contour[1]);
    // assign contour calculation for all points in original contour
    int i;
    for ( i = 1; i < contour.size() - 1; ++i ) {
        double dxi = (contour[i].x - contour[i-1].x);
        double dxi1 = (contour[i+1].x - contour[i].x);
        double dyi = (contour[i].y - contour[i-1].y);
        double dyi1 = (contour[i+1].y - contour[i].y);
        double dsi = sqrt(pow(dxi,2) + pow(dyi,2));
        double dsi1 = sqrt(pow(dxi1,2) + pow(dyi1,2));
        double ds = (dsi + dsi1) / 2.0;

        double c = (1/ds) * sqrt(pow((dxi/dsi) - (dxi1/dsi1), 2) +
                             pow((dyi/dsi) - (dyi1/dsi1), 2));
        c = pow(c, 2);
        curvature.push_back(c);
    }

    normalize(curvature);

    return curvature;
}

/* NOT WORKING.. debug? */
vector<double> curvature_k(vector<Point> contour) {
    vector<double> curvature;
    // adjust contour to handle start and end positions
    contour.insert(contour.begin(), contour[contour.size() - 1]);
    contour.push_back(contour[1]);
    // assign contour calculation for all points in original contour
    int i;
    for ( i = 1; i < contour.size() - 1; ++i ) {
        double dx = (contour[i].x - contour[i-1].x);
        double d2x = (contour[i-1].x - 2*contour[i].x + contour[i+1].x);
        double dy = (contour[i].y - contour[i-1].y);
        double d2y = (contour[i-1].y - 2*contour[i].y + contour[i+1].y);
        double k = (dx*d2y - dy*d2x) / pow(dx*dx + dy*dy, (3/2));

        double c = pow(k, 2);
        curvature.push_back(c);
    }

    normalize(curvature);

    return curvature;
}



