#include "MotionEstimator.h"

MotionEstimator::MotionEstimator(Mat& roi, Point offsetPt)
	: frameROI(roi)
	, offsetPt(offsetPt)
{
	cvtColor(frameROI, frameROI_gray, CV_BGR2GRAY);

    // Parameters for Shi-Tomasi algorithm
    qualityLevel = 0.1;
    minDistance = 2;

	// Affine transformation needs 3 pairs of points
	inputPts.resize(3);
	outputPts.resize(3);

	// Affine transformation needs 4 pairs of points
	//inputPts.resize(4);
	//outputPts.resize(4);
}

// Feature point detection
void MotionEstimator::detectFeaturePoints()
{
    cout << "Detecting Features..." << endl;
    // Detect the features
    cv::goodFeaturesToTrack(frameROI_gray,  // ROI selected by user
                            features,       // output detected features points
                            MAX_FEATURES,   // max number of features
                            qualityLevel,   // quality level
                            minDistance);   // min distance between 2 features
    cout << "\tNumber of features detected = " << features.size() << endl;

	// Offset locations of all feature points by location of top-left corner of ROI
	for (size_t i = 0; i < features.size(); i++)
	{
		features[i].x += offsetPt.x;
		features[i].y += offsetPt.y;
	}
}

// Perform entire Feature Detection pipeline on given frame
void MotionEstimator::performFeatureDetection(Mat& frameIn)
{
	frame = frameIn.clone();
	cvtColor(frame, gray, CV_BGR2GRAY);

    // For first frame of sequence
    if (grayRef.empty())
	{
		frame.copyTo(refFrame);    // Use frame 1 as reference frame (for affine transform)
        gray.copyTo(grayRef);		
	}

    // If number of feature points is insufficient, detect more
    if (points[0].size() <= 10)
    {
        // Detect feature points
        detectFeaturePoints();

        // Add detected features to currently tracked features
        points[0].insert(points[0].end(), features.begin(), features.end());
    }
	
    // Track features using Lucas-Kanade method
    //cout << "Calculating Optical Flow..." << endl;
    calcOpticalFlowPyrLK(grayRef, gray,   // reference frame, current frame
                         points[0],       // input point positions in reference frame
                         points[1],       // output point positions in current frame
                         status,          // tracking success
                         err);            // tracking error
	
	int k = 0;
    //for (size_t i = 0; i < points[1].size(); i++)
	for (size_t i = 0; i < 3; i++)  // check every feature point
    {
		//if ( status[i] == 0 )
  //          continue;
  //      // Don't consider points rejected by optical flow
  //      //            (status[i] &&
		//// Or points that did not move
  //      //            (abs(points[0][i].x - points[1][i].x) +
  //      //            (abs(points[0][i].y - points[1][i].y)) > 2));
  //      numValid++;

		inputPts[k] = points[0][i];
		outputPts[k] = points[1][i];
		
		//cout << inputPts[k] << endl;
		//cout << outputPts[k] << endl <<endl;
		k++;


	    Point p,q;
        p.x = (int) points[0][i].x;
        p.y = (int) points[0][i].y;
        q.x = (int) points[1][i].x;
        q.y = (int) points[1][i].y;

        double angle;
        angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        double dist;  // distance between points
        dist = sqrt( square(p.y - q.y) + square(p.x - q.x) );

        // Lengthen the line by a factor of 3 (else arrows are short because of barely any motion between frames)
        q.x = (int) (p.x - 3 * dist * cos(angle));
        q.y = (int) (p.y - 3 * dist * sin(angle));

        // Draw a circle on feature points, and line for motion vectors
        // line thickness = 1, CV_AA = AntiAliased drawing, 0 = no fractional bits in center coordinate or radius
        cv::line(frame, p, q, cv::Scalar(255,0,0), 1, CV_AA, 0);		// blue line
        cv::circle(frame, points[1][i], 3, cv::Scalar(0,0,255), -1, 8); // red point, radius = 3     
    }
    imshow("Feature Tracking", frame );
    //cout << "\tNumber of features points rejected : " << points[1].size()-numValid << endl;
}

void MotionEstimator::findAffine()
{	
	transform = getAffineTransform(inputPts, outputPts);
}


Mat MotionEstimator::applyAffine(Mat& frameIn)
{	
	findAffine();
	Mat input = frameIn.clone();
	// IMPORTANT: The affine transformation estimated above is a mapping from reference frame
	// to current frame. In order to correct this in the current frame, 
	// the INVERSE mapping needs to be applied
	warpAffine(input, warpedFrame, transform, refFrame.size(), WARP_INVERSE_MAP);
	return warpedFrame;
}

void MotionEstimator::findPerspective()
{	
	transform = getPerspectiveTransform(inputPts, outputPts);
}

Mat MotionEstimator::applyPerspective(Mat& frameIn)
{	
	findPerspective();
	Mat input = frameIn.clone();
	// IMPORTANT: The perspective transformation estimated above is a mapping from reference frame
	// to current frame. In order to correct this in the current frame, 
	// the INVERSE mapping needs to be applied
	warpPerspective(input, warpedFrame, transform, refFrame.size(), WARP_INVERSE_MAP);
	return warpedFrame;
}