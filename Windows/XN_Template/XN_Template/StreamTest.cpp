#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <fstream>
#include "XN_net.hpp"
#include "XN_yolo.hpp"

//using namespace std;
using namespace cv;

// default command line args
std::string keys =
"{ tracking_target t | bottle | camera servos will try to keep this object in the center of the frame }"
"{ framework f | | Optional name of an origin framework of the model. Detect it automatically if it does not set. }"
"{ classes     | object_detection_classes_yolov3.txt | Optional path to a text file with names of classes to label detected objects. }"
"{ config      | D:/Code/darknet/cfg/yolov4.cfg | }"
"{ model       | D:/Code/darknet/yolov4.weights | }";


int main(int argc, const char** argv)
{
    CommandLineParser parser(argc, argv, keys);

    xn::yolo::YoloInfo yo(Size(256, 256));
    xn::yolo::TrackingInfo track(parser.get<String>("tracking_target"), parser.get<String>("classes"));
    dnn::Net net;
    xn::yolo::create_dnn( yo, net,
                          "D:/Code/darknet/yolov3.weights",
                          "D:/Code/darknet/cfg/yolov3.cfg");

    // Create a window
    static const std::string kWinName = "Deep learning object detection in OpenCV";
    namedWindow(kWinName, WINDOW_NORMAL);

    // Open a video file or an image file or a camera stream.
    VideoCapture cap;
    cap.open(0);
    //xnnet::VideoStream cap(8080);

    // Process frames.
    Mat frame, blob;
    while (true)
    {
        cap >> frame;
 /*       frame = cap.getLastFrame();
        if (frame.empty())
            continue;*/

        xn::yolo::preprocess(frame, yo, net);

        std::vector<Mat> outs;
        net.forward(outs, yo.out_names);

        xn::yolo::postprocess(frame, outs, net, yo, track);

        // Put efficiency information.
        std::vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        std::string label = format("Inference time: %.2f ms", t);
        putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
        imshow(kWinName, frame);     

        // quit on escape
        if (waitKey(10) == 27)
            break;

    }
    return 0;
}