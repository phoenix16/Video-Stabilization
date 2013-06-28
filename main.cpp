/*
 * Developer : Prakriti Chintalapoodi - c.prakriti@gmail.com 
*/

#include "MotionEstimator.h"

#define PI 3.1415926535;
#define NUM_FRAMES_TO_PROCESS  200

// Global Variables
Mat frame1, frame_roi;
Point pt1, pt2;
bool roi_capture = false;
bool got_roi = false;

// Function Headers
void mouse_click(int event, int x, int y, int flags, void *param);

int main(int argc, char* argv[])
{
    if (argc != 2)
    string pathToImages = "/VideoStabilization/data";

    VideoWriter outputVideo1, outputVideo2;
    outputVideo1.open("source.avi", 1145656920, 30, Size(480, 360), true);   // X-VID MPEG4 code = 1145656920
    outputVideo2.open("stabilized.avi", 1145656920, 30, Size(480, 360), true);

    if (!outputVideo1.isOpened() || !outputVideo2.isOpened())
    {
        cerr << "!!! ERROR: Video Writer not initialized\n" << endl;
        exit(1);
    }

    // Initialize cumulative Homography matrix to identity matrix
    Mat Hcumulative = Mat::eye(3,3,CV_32FC1);

    vector<string> imagesVec = read_directory(pathToImages);

    for (int i = 2; i < imagesVec.size(); i++)
    {
        Mat H, warpedImage2;
        Mat image1 = imread(pathToImages + "/" + imagesVec[i]);
        Mat image2 = imread(pathToImages + "/" + imagesVec[i+1]);

        if (!image1.empty() || !image2.empty())
        {
            //            imshow("out", image1);
            //            imshow("out1", image2);
            findHomographyMatrix(image1, image2, H);

            H.convertTo(H, CV_32FC1);
//            cout << "\nHomography between images " << imagesVec[i] << " and " << imagesVec[i+1] << " is :\n" << H << endl;

            Hcumulative = H * Hcumulative;
//            cout << "\nCumulative Homography till image " << imagesVec[i+1] << " is :\n" << Hcumulative << endl;

            warpPerspective(image2, warpedImage2, Hcumulative, image2.size());

//            imshow("source" + imagesVec[i+1], image2);
//            imshow("warp" + imagesVec[i+1], warpedImage2);

            outputVideo1.write(image1);
            if (i == imagesVec.size() - 1)  // Write last frame of source clip
            {  outputVideo1.write(image2); }
            if (i == 2)               // Write first frame of stabilized clip
            {  outputVideo2.write(image1); }
            outputVideo2.write(warpedImage2);

        }
        else
        {
            cerr << "Warning: Could not read image " <<  imagesVec[i] << endl;
            exit(1);
        }
    }

    cout << "Finished writing output videos" << endl;
    waitKey(0);
    return 0;
}



void findHomographyMatrix(Mat& image1, Mat& image2, Mat& homography)
{
    // Detect the keypoints using SURF Detector
    int minHessian = 400;
    SurfFeatureDetector detector(minHessian);
    std::vector<KeyPoint> keypoints_image1, keypoints_image2;
    detector.detect( image1, keypoints_image1 );
    detector.detect( image2, keypoints_image2 );

    // Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat descriptors_image1, descriptors_image2;
    extractor.compute( image1, keypoints_image1, descriptors_image1 );
    extractor.compute( image2, keypoints_image2, descriptors_image2 );

    // Match descriptor vectors using FLANN matcher
    // FLANN matching serves as initialization to the RANSAC feature matching (future step)
    // FLANN finds the nearest neighbors of keypoints in left image present in the right image
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_image1, descriptors_image2, matches );

    double max_dist = 0, min_dist = 100;

    // Find max and min distances between keypoints
    for (int i = 0; i < descriptors_image1.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    // Use only "good" matches (i.e. whose distance is less than 3*min_dist ) to
    // construct Homography (Projective Transformation)
    std::vector< DMatch > good_matches;
    for (int i = 0; i < descriptors_image1.rows; i++)
    {
        cout << "Usage: ./Video-Stabilization <Input Video file>" << endl;
        return -1;
    }

	// Open destabilized input video
    VideoCapture inputVideo(argv[1]);
    if (!inputVideo.isOpened())
    {
        cout << "Error opening Input Video " << argv[1] << endl;
        return -1;
    }

	// Get the properties of input video to write output video
    double fps = inputVideo.get(CV_CAP_PROP_FPS);
    int delay = 1000/fps;    // time in ms between successive frames = 1000/fps
    cout << "FPS = " << fps << endl;
    cout << "Number of frames in input video = " << static_cast<int>(inputVideo.get(CV_CAP_PROP_FRAME_COUNT)) << endl;
	cout << "Number of frames to process = " << NUM_FRAMES_TO_PROCESS << endl;
	int CodecType = static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC));
	int width = static_cast<int>(inputVideo.get(CV_CAP_PROP_FRAME_WIDTH));
	int height = static_cast<int>(inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));

	// Create stabilized output video
    VideoWriter outputVideo;
	string outFile = string(argv[1]);
	outFile.insert(outFile.length()-4,"_stabilized");
    outputVideo.open(outFile, CodecType, fps, Size(width, height), true);   // Same codec as input video
    if (!outputVideo.isOpened())
    {
        cerr << "!!! ERROR: Video Writer not initialized\n" << endl;
        exit(1);
    }

	inputVideo >> frame1;     // read first frame only
	if(!frame1.empty())
    {
        // Collect the ROI of the object, store it in img_object
        cout << "Click and drag to select Region of Interest" << endl;
        namedWindow("Frame 1: Capture ROI", 1);
        imshow("Frame 1: Capture ROI", frame1);
        setMouseCallback("Frame 1: Capture ROI", mouse_click, 0);

        waitKey(0);

        // close the window after ROI is saved
        destroyWindow("Frame 1: Capture ROI");
        destroyWindow("ROI");
    }

    // Set position in video back to beginning
    inputVideo.set(CV_CAP_PROP_POS_FRAMES, 0);
	
    // Process every frame of video only when frame ROI has been selected
    if (!frame_roi.empty())
    {
		// Create instance of Motion Estimator
		MotionEstimator m(frame_roi, pt1);

		Mat frame, stabilizedFrame;
		bool stop(false);
		int frameNum = 0;
		
		while(!stop)
		{
			inputVideo >> frame;            // read current frame
			if( frame.empty()) break;       // check if at end
			frameNum++;
			imshow("Input Video (without Video Stabilization)", frame);
			
			// Perform Optical Flow and detect feature points
			m.performFeatureDetection(frame);

			// Apply the affine transform to current frame
			stabilizedFrame = m.applyAffine(frame);
			//stabilizedFrame = m.applyPerspective(frame);

			// Display Stabilized video
			imshow("Stabilized Video", stabilizedFrame);
			imwrite("stab.png", stabilizedFrame);

			// Write Stabilized video
			//outputVideo.write(stabilizedFrame);

			if (inputVideo.get(CV_CAP_PROP_POS_FRAMES) == NUM_FRAMES_TO_PROCESS)
			{
				cout << "End" << endl;
				break;
			}
			// introduce delay or press key to stop
			if (waitKey(delay) >= 0)
				stop = true;
		}
	}

    waitKey(0);
    return 0;
}



// Records mouseclick events to get x-y coordinates of ROI
// Store coordinates in global variables pt1, pt2.
void mouse_click(int event, int x, int y, int flags, void *param)
{
    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
    {
        cout << "Mouse Left Button Pressed" << endl;
        if (!roi_capture)
        {
            pt1.x = x;
            pt1.y = y;
        }
        else
        {
            cout << "ROI already acquired" << endl;
        }
        break;
    }
    case CV_EVENT_LBUTTONUP:
    {
        if (!got_roi)
        {
            cout << "Mouse Left Button released" << endl;
            pt2.x = x;
            pt2.y = y;

            Mat roi(frame1, Rect(pt1, pt2));
            roi.copyTo(frame_roi);
            cout << "ROI acquired" << endl;
            namedWindow("ROI", 1);
            imshow("ROI", roi);
            got_roi = true;
            cout << "Press any key to close ROI windows and start video stabilization" << endl;
        }
        else
        {
            cout << "ROI already acquired" << endl;
        }
        break;
    }
    }
}
