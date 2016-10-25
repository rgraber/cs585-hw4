#include "hw4.hpp"

void drawCurvatureIntensity(Mat& src, Mat& dst) {
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    // use contour with most points
    vector<Point> object = contours[0];
    for ( int i = 1; i < contours.size(); ++i ) {
        if ( contours[i].size() > object.size() ) {
            object = contours[i];
        }
    }

    // get curvature vector
    vector<double> curve = curvature_dtds(object);
    //vector<double> curve = curvature_vss(object);
    //vector<double> curve = curvature_k(object);
    cout << "Average: " << mean(curve)[0] << endl;

    int intensity = 0;
    object.insert(object.begin(), object[object.size() - 1 ]);
    for ( int i = 1; i < object.size(); ++i ) {
        intensity = max(0, (int)(255*curve[i-1]));
        line(dst, object[i-1], object[i],
                Scalar(intensity,intensity,intensity), 4);
        //circle(dst, object[i-1], 5, Scalar(0,0,255), 1);
    }
}

void drawVector(Mat& dst, vector<Point> points, Scalar color) {
    int intensity;
    int cutoff = 20;
    for ( int i = 0; i < points.size(); ++i ) {
        if ( i < cutoff ) {
            color = Scalar(255,0,0);
        } else if ( i > points.size() - cutoff ) {
            color = Scalar(0,0,255);
        } else {
            color = Scalar(0,255,0);
        }
        circle(dst, points[i], 5, color, 1);
    }
}

int main(int argc, char** argv)
{
    Mat src;
    if (argc < 2) {
        src = imread("data/orientation_test.png", IMREAD_GRAYSCALE);
    } else {
        src = imread(argv[1], IMREAD_GRAYSCALE);
    }

    //Mat buffer = Mat::zeros(src.size(), src.type());
    //skeletonize(src, buffer);
    Mat buffer = src.clone();

    vector<Point> vectorized_eel;
    vectorized_eel = skeletonToContour(buffer);

    Mat dst = buffer.clone();
    cvtColor(dst, dst, CV_GRAY2BGR);
    drawVector(dst, vectorized_eel, Scalar(0,0,255));

    //drawCurvatureIntensity(src, dst);

    imshow("Output", dst);
    waitKey(0);
}

