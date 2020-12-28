#include <stdio.h>
#include <float.h>
#include <limits.h>

#include <iostream>
#include <chrono>
#include <ctime>

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#define DETECT_EYES 0

using namespace std;
using namespace cv;
double detectAndWrite( Mat frame );
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
int main( int argc, const char** argv )
{
    CommandLineParser parser(argc, argv,
                             "{help h||}"
                             "{image|test.jpg|Input image file.}"               
                             "{iters|10|# of tests to run.}");               
    parser.about("Using the cv::CascadeClassifier class to detect objects (Face + eyes) in an image.\n");

    cout << "Face detection with cv::CascadeClassifier class\n";

    String face_cascade_name = samples::findFile( "haarcascade_frontalface_alt.xml" );
    String eyes_cascade_name = samples::findFile( "haarcascade_eye_tree_eyeglasses.xml" );
   
    // load data models
    if( !face_cascade.load( face_cascade_name ) )
    {
        cout << "--(!)Error loading face cascade\n";
        return -1;
    };
    if( !eyes_cascade.load( eyes_cascade_name ) )
    {
        cout << "--(!)Error loading eyes cascade\n";
        return -1;
    };

    Mat frame = imread( samples::findFile( parser.get<String>("image") ), IMREAD_COLOR );
    cout << "loaded input file '" << parser.get<String>("image") << "'\n\n";

#if DETECT_EYES
    cout << "Eye Detection: On\n";
#else
    cout << "Eye Detection: Off\n";
#endif
 
    // load input image
    double total_dur = 0;
    double lowest_dur = DBL_MAX;
    double highest_dur = 0; 
    int iters =  parser.get<int>("iters");
    for ( int i = 0; i < iters; i++ ) {
        Mat frame = imread( samples::findFile( parser.get<String>("image") ), IMREAD_COLOR );
        double t = detectAndWrite( frame );
        if ( t < lowest_dur ) lowest_dur = t; 
        if ( t > highest_dur ) highest_dur = t; 
        total_dur += t;
    }
    
    printf("avgerage runtime:      %f seconds\n", total_dur/iters);
    printf("highest  runtime:      %f seconds\n", highest_dur);
    printf("lowest   runtime:      %f seconds\n", lowest_dur);
    printf("total (%d iterations): %f seconds\n", iters, total_dur);
    return 0;
}
double detectAndWrite( Mat frame )
{
    // start timer
    auto t1 = std::chrono::high_resolution_clock::now();
    
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( frame_gray, faces );
    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4 );
        Mat faceROI = frame_gray( faces[i] );
        
#if DETECT_EYES
        // detect eyes
        std::vector<Rect> eyes;
        eyes_cascade.detectMultiScale( faceROI, eyes );
        for ( size_t j = 0; j < eyes.size(); j++ )
        {
            Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
            int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            circle( frame, eye_center, radius, Scalar( 255, 0, 0 ), 4 );
        }
#endif
    }

    // get processing time
    auto t2 = std::chrono::high_resolution_clock::now();
    auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "Runtime: " << time_span.count() << " seconds.\n";

    // show me what you got
    imwrite( "face_circled.jpg", frame );
    return time_span.count();
}