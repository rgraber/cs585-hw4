#include "hw4.hpp"

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
    Mat buffer = Mat::zeros(src.size(), src.type());

    // Find a single point on the border of the object
    Point s = borderPoint(src, obj_id);

    int found = obj_id;
    vector<Point> dividingLine;
    vector<Point> searchSpace;
    vector<Point> leftSearch;
    vector<Point> rightSearch;

    Point p;
    double theta = orientation(src, obj_id);
    if (theta < 2.35 && theta > 0.785 ) {
        // Assumes a vertical orientation

        // Find the dividing line from the start point
        for ( int col = s.x; col < src.cols - s.x; ++col ) {
            if ( src.at<uchar>(s.y, col) == obj_id ) {
                p = Point(col, s.y);
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
            buffer.at<uchar>(p.y, p.x) = found;
            if ( p.y < s.y ) {
                leftSearch.push_back(p);
            } else if ( p.y > s.y ) {
                rightSearch.push_back(p);
            }
        }

    } else {
        // Assumes a horizontal orientation

        // Find the dividing line from the start point (2)
        for ( int row = s.y+1; row < src.rows - s.y; ++row ) {
            if ( src.at<uchar>(row, s.x) == obj_id ) {
                p = Point(s.x, row);
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
            buffer.at<uchar>(p.y, p.x) = found;
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
                if ( src.at<uchar>(p.y, p.x) == found &&
                            buffer.at<uchar>(p.y, p.x) != found ) {
                    buffer.at<uchar>(p.y, p.x) = found;
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
void skeletonize(Mat& src, Mat& dst, int obj_id) {

    Mat buffer = Mat::zeros(src.size(), src.type());
    distanceToBackground(src, buffer, obj_id);

    int row, col;
    int dist;
    for ( row = 0; row < src.rows; ++row ) {
        for ( col = 0; col < src.cols; ++col ) {
            if ( src.at<uchar>(row, col) == obj_id ) {
                dist = buffer.at<uchar>(row, col);
                if (buffer.at<uchar>(row-1, col) <= dist &&
                    buffer.at<uchar>(row+1, col) <= dist &&
                    buffer.at<uchar>(row, col-1) <= dist &&
                    buffer.at<uchar>(row, col+1) <= dist) {
                    dst.at<uchar>(row, col) = 255;
                }
            }
        }
    }
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
                if ( src.at<uchar>(p.y - dist, p.x) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // bottom
            if ( p.y + dist < src.rows ) {
                if ( src.at<uchar>(p.y + dist, p.x) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // left
            if ( p.x - dist >= 0 ) {
                if ( src.at<uchar>(p.y, p.x - dist) == 0 ) {
                    break;
                }
                continueSearch = true;
            }
            // right
            if ( p.x + dist < src.cols ) {
                if ( src.at<uchar>(p.y, p.x + dist) == 0 ) {
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
        dst.at<uchar>(p.y, p.x) = dist;
    }
}

Point borderPoint(Mat& src, int obj_id) {
    Point p;
    int row, col;
    for ( row = 0; row < src.rows; ++row ) {
        for ( col = 0; col < src.cols; ++col ) {
            if (src.at<uchar>(row, col) == obj_id) {
                p = Point(col, row);
                return p;
            }
        }
    }
    return p;
}

