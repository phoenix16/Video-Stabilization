#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <dirent.h>
#include <sys/types.h>

using namespace cv;
using namespace std;


vector <string> read_directory(const string& path = string());
void writeVideo(const string pathToImages, const string videoName, double fps, int width, int height);

int main(int argc, char* argv[])
{
    string pathToImages = "/home/prakriti/projects/OpenCV/Projects/Video-Stabilization/in";

    writeVideo(pathToImages, "output.avi", 30, 480, 360);

    return 0;
}




// Function to create a video from a set of PNG images
// Provide directory path to images, desired frame rate, width and height of source images
void writeVideo(const string pathToImages, const string videoName, double fps, int width, int height)
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
//
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
