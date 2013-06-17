#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <dirent.h>
#include <sys/types.h>

using namespace cv;
using namespace std;

void findHomographyMatrix(Mat& image1, Mat& image2, Mat& homography);

vector <string> read_directory(const string& path = string());
void writeVideo(const string pathToImages, const string videoName, double fps, int width, int height);

int main()
{
    string pathToImages = "/home/prakriti/projects/Datasets/VideoStabilization/cam2";

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
        if (matches[i].distance < 3*min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }

    // Isolate the matched keypoints in each image
    std::vector<Point2f> image1_matchedKPs;
    std::vector<Point2f> image2_matchedKPs;

    for (size_t i = 0; i < good_matches.size(); i++ )
    {
        image1_matchedKPs.push_back( keypoints_image1[ good_matches[i].queryIdx ].pt );
        image2_matchedKPs.push_back( keypoints_image2[ good_matches[i].trainIdx ].pt );
    }

    // Find the Homography relating image2 and image1
    homography = findHomography( Mat(image2_matchedKPs), Mat(image1_matchedKPs), CV_RANSAC );


    // Warp image2 to image1's space using the Homography just constructed
    //    Mat image2Warped;  // warped image has twice the width to account for overlap
    //    warpPerspective(image2, image2Warped, H, Size(image2.cols*2, image2.rows), INTER_CUBIC);

    //    panorama = image2Warped.clone();
    // Overwrite image1 on left end of final panorma image
    //    Mat roi(panorama, Rect(0, 0, image1.cols, image1.rows));
    //    image1.copyTo(roi);
}





// Function to create a video from a set of PNG images
// Provide directory path to images, desired frame rate, width and height of source images
void writeImages2Video(const string pathToImages, const string videoName, double fps, int width, int height)
{
    VideoWriter outputVideo;
    outputVideo.open(videoName, 1145656920, fps, Size(width, height), true);   // X-VID MPEG4 code = 1145656920

    if (!outputVideo.isOpened())
    {
        cerr << "!!! ERROR: Video Writer not initialized\n" << endl;
        exit(1);
    }

    vector<string> a = read_directory(pathToImages);


    for (int i = 0; i < a.size(); i++)
    {
        string imagePath = pathToImages + "/" + a[i];
        size_t found = imagePath.find(".png", imagePath.length() - 4);  // Search for images with .png extension
        if (found!=string::npos)
        {
            cout << "\tProcessing file " << a[i] << endl;

            Mat image = imread(imagePath);
            if (!image.empty())
            {
                outputVideo.write(image);
            }
            else
            {
                cerr << "Warning: Could not read image: " <<  a[i] << endl;
            }
        }
    }
    cout << "\n\tFinished writing output video" << endl;
}


// Function to return an ASCII-sorted vector of filename entries in a given directory.
//   If no path is specified, the current working directory is used.
//
//   Always check the value of the global 'errno' variable after using this
//   function to see if anything went wrong. (It will be zero if all is well.)
vector <string> read_directory(const string& path)
{
    vector <string> result;
    dirent* de;
    DIR* dp;

    dp = opendir( path.empty() ? "." : path.c_str() );
    if (dp)
    {
        while (true)
        {
            de = readdir( dp );
            if (de == NULL) break;
            result.push_back( string( de->d_name ) );
        }
        closedir( dp );
        sort( result.begin(), result.end() );
    }
    return result;
}

