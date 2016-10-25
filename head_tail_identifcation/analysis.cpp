#include "hw4.hpp"

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

