//
// Developer : Prakriti Chintalapoodi - c.prakriti@gmail.com 
//

#ifndef MotionEstimator_H
#define MotionEstimator_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <string>
#include <iostream>

#define MAX_FEATURES 400

using namespace cv;
using namespace std;

inline static double square(int a)
{
    return a * a;
}

class MotionEstimator
{
private:	
	Mat frame;
    Mat gray;			// current grayscale frame
    Mat grayRef;		// Frame 1 grayscale
	Mat refFrame;		// Use frame 1 as reference frame
	Mat warpedFrame;	// output after affine transform
	Mat frameROI, frameROI_gray;
	Point offsetPt;

	Mat transform;				
	vector<Point2f> inputPts;	 
	vector<Point2f> outputPts;

    vector<Point2f> points[2];  // Tracked features from 0->1
    vector<Point2f> features;   // detected features
    vector<uchar> status;      // status of tracked features
    vector<float> err;         // error in tracking

    int maxCorners;         // max number of corners in goodFeaturesToTrack function
    double qualityLevel;    // Parameters for Shi-Tomasi algorithm
    double minDistance;     // Parameters for Shi-Tomasi algorithm	    
	
	void detectFeaturePoints();
	void findAffine();
	void findPerspective();

public:
    MotionEstimator(Mat &roi, Point offsetPt);
    void performFeatureDetection(Mat& frameIn);

	Mat applyAffine(Mat& frameIn);
	Mat applyPerspective(Mat& frameIn);
};

#endif // MotionEstimator_H
