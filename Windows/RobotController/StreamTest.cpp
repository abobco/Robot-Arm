#define _CRT_SECURE_NO_WARNINGS

#include "xn_net.hpp"
#include "xn_yolo.hpp"
#include "cam_calib.hpp"
#include "xn_homo.hpp"

#include <stdlib.h> 
#include <iostream>
#include <fstream>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/opencv.h>

#define PORT 8080 

using namespace cv;
using namespace dnn;

// default command line args
std::string keys =
"{ cam_intrinsics | out_camera_data_small.xml | path to camera intrinsics data }"
"{ tracking_target t | bottle | camera servos will try to keep this object in the center of the frame }"
"{ framework f | | Optional name of an origin framework of the model. Detect it automatically if it does not set. }"
"{ classes     | object_detection_classes_yolov3.txt | Optional path to a text file with names of classes to label detected objects. }"
"{ config      | D:/Code/darknet/cfg/yolov4.cfg | }"
"{ model       | D:/Code/darknet/yolov4.weights | }";

Mat frame;

Point2i clickpt(0,0);
Point3d worldpt;

bool found = false;
Mat H;
xn::homo::HomoTransform trans;

void mouseClickCallback(int evt, int x, int y, int flags, void* userdata) {
    if (evt == EVENT_LBUTTONDOWN) {
        cout << x << ", " << y << "\n";
        clickpt = Point2i(x, y);
    }
}

int main(int argc, const char** argv)
{
    //int blockSize = 256;
    //int r = 4;
    //int c = 5;
    ////int imageSize = blockSize * ;
    //Mat chessBoard(r*blockSize, c*blockSize, CV_8UC3, Scalar::all(0));
    //unsigned char color = 0;

    //for (int i = 0; i < c*blockSize; i = i + blockSize) {
    //    color = ~color;
    //    for (int j = 0; j < r*blockSize; j = j + blockSize) {
    //        Mat ROI = chessBoard(Rect(i, j, blockSize, blockSize));
    //        ROI.setTo(Scalar::all(color));
    //        color = ~color;
    //    }
    //}
    //imshow("Chess board", chessBoard);
    //imwrite("chessboard1.png", chessBoard);
    //waitKey(0);
    //return 0;

    //camcalib::calibrate_camera("in_VID5.xml");
    //return 0;

    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
    dlib::shape_predictor predictor;
    dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;

    CommandLineParser parser(argc, argv, keys);

    xn::yolo::YoloInfo yo(Size(256, 256));
    xn::yolo::TrackingInfo track(parser.get<String>("tracking_target"), parser.get<String>("classes"));
    Net net = Net();
    xn::yolo::create_dnn( yo, net,
                          parser.get<String>("model"),
                          parser.get<String>("config"));

    // Create a window
    static const std::string kWinName = "Deep learning object detection in OpenCV";
    namedWindow(kWinName);

    // Open a video file or an image file or a camera stream.
    //VideoCapture cap;
    //cap.open(0);
    xn::VideoStream cap(8080);

    setMouseCallback(kWinName, mouseClickCallback, NULL);

    // Process frames.
    Size pattern_size = Size(4, 3);
    float square_size = 21;

    // face pose result
    cv::Mat rotation_vec;                           //3 x 1
    cv::Mat rotation_mat;                           //3 x 3 R
    cv::Mat translation_vec;                        //3 x 1 T
    cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
    cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

    std::vector<cv::Point2d> facepoints_2d;
    //reprojected 2D points
    std::vector<cv::Point2d> reprojectdst;
    reprojectdst.resize(8);
    cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
    cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
    cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

    FileStorage fs(samples::findFile(parser.get<String>("cam_intrinsics")), FileStorage::READ);
    Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    std::vector<Point2f> corners;
    while (true)
    {
        frame = cap.getLastFrame();
        Mat outframe = frame.clone();
        if (frame.empty())
            continue;

        dlib::cv_image<dlib::bgr_pixel> cimg(frame);
        // Detect faces
        std::vector<dlib::rectangle> faces = detector(cimg);
        if (faces.size() > 0) {
            //track features
            dlib::full_object_detection shape = predictor(cimg, faces[0]);

            //draw features
            for (unsigned int i = 0; i < 68; ++i)
            {
                cv::circle(outframe, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
            }  

            //fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/
            facepoints_2d.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
            facepoints_2d.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
            facepoints_2d.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
            facepoints_2d.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner
            facepoints_2d.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
            facepoints_2d.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
            facepoints_2d.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
            facepoints_2d.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner
            facepoints_2d.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
            facepoints_2d.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
            facepoints_2d.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
            facepoints_2d.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
            facepoints_2d.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner
            facepoints_2d.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner

            //calc pose
            cv::solvePnP(xn::homo::Face3D, facepoints_2d, cameraMatrix, distCoeffs, rotation_vec, translation_vec);

            //reproject
            cv::projectPoints(xn::homo::facecube_src, rotation_vec, translation_vec, cameraMatrix, distCoeffs, reprojectdst);

            //draw axis
            cv::line(outframe, reprojectdst[0], reprojectdst[1], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[1], reprojectdst[2], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[2], reprojectdst[3], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[3], reprojectdst[0], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[4], reprojectdst[5], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[5], reprojectdst[6], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[6], reprojectdst[7], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[7], reprojectdst[4], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[0], reprojectdst[4], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[1], reprojectdst[5], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[2], reprojectdst[6], cv::Scalar(0, 0, 255));
            cv::line(outframe, reprojectdst[3], reprojectdst[7], cv::Scalar(0, 0, 255));

            //calc euler angle
            cv::Rodrigues(rotation_vec, rotation_mat);
            cv::hconcat(rotation_mat, translation_vec, pose_mat);
            cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

            Point2i labelPos(facepoints_2d[0].x - 50, facepoints_2d[0].y - 50);
            cv::putText(outframe, cv::format("(%.2f, %.2f, %.2f)", translation_vec.at<double>(0), translation_vec.at<double>(1), translation_vec.at<double>(2)), labelPos, cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255));

            if (xn::arm_state == -1) {
                xn::send_int(cap.client_sock, translation_vec.at<double>(0));
                xn::send_int(cap.client_sock, translation_vec.at<double>(1));
                xn::send_int(cap.client_sock, translation_vec.at<double>(2));
            }

            facepoints_2d.clear();
        }

        /*cap >> frame;*/
        //normalize(frame, frame, 0, 255, NORM_MINMAX);
        std::vector<Point2f> cornersTemp;
        if (findChessboardCorners(frame, pattern_size, cornersTemp)) {
            found = true;
            corners = cornersTemp;
            // corner points in the object frame
            std::vector<Point3f> objectPoints;
            for (int i = 0; i < pattern_size.height; i++)
                for (int j = 0; j < pattern_size.width; j++)
                    objectPoints.push_back(Point3f(float(j * square_size),
                        float(i * square_size), 0));

            std::vector<Point2f> objectPointsPlanar;
            for (size_t i = 0; i < objectPoints.size(); i++)
                objectPointsPlanar.push_back(Point2f(objectPoints[i].x, objectPoints[i].y));

            vector<Point2f> imagePoints;
            undistortPoints(corners, imagePoints, cameraMatrix, distCoeffs);

            // estimate homography matrix
            H = findHomography(objectPointsPlanar, imagePoints);

            // estimate camera pose
            /*xn::homo::HomoTransform trans*/;
            xn::homo::GetHomoTransform(H, trans);
            trans.tvec.at<double>(1) += 20;
            std::vector<Point3f> axes;
            float ax_len = square_size * 2;
            axes.push_back(objectPoints[0]);
            axes.push_back(objectPoints[0] + Point3f(ax_len, 0, 0));
            axes.push_back(objectPoints[0] + Point3f(0, ax_len, 0));
            axes.push_back(objectPoints[0] + Point3f(0, 0, ax_len));

            std::vector<Point2f> axesProjected;
            std::vector<Point2f> cornersProjected;
            projectPoints(objectPoints, trans.rotation, trans.tvec, cameraMatrix, distCoeffs, cornersProjected);
            projectPoints(axes, trans.rotation, trans.tvec, cameraMatrix, distCoeffs, axesProjected);

            Point2f obj_cen(0, 0);
            for (Point2f& p : cornersProjected) {
                obj_cen += p;
            }
            obj_cen.x /= objectPoints.size();
            obj_cen.y /= objectPoints.size();

            Scalar axcolors[] = {
                Scalar(255,0,0),
                Scalar(0,255,0),
                Scalar(0,0,255)
            };


            for (int i = 1; i < axesProjected.size(); i++) {
                line(outframe, axesProjected[0], axesProjected[i], axcolors[i-1], 2);
            }
        }
        if (found /*&& clickpt.x > 0*/ ) {

            /*   for (Point3f& p : objectPoints) {
                   p.z = -50;
               }*/

               // yolo dnn object detection
            xn::yolo::preprocess(frame, yo, net);
            std::vector<Mat> outs;
            net.forward(outs, yo.out_names);
            xn::yolo::postprocess(frame, outs, net, yo, track);
            std::vector<double> layersTimes;
            double freq = getTickFrequency() / 1000;
            double t = net.getPerfProfile(layersTimes) / freq;
            std::string label = format("Inference time: %.2f ms", t);
            putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));

            if (track.in_frame) {
            xn::homo::inverseProjectPoint(track.base, worldpt, cameraMatrix, trans);
            //xn::homo::inverseProjectPoint(clickpt, worldpt, cameraMatrix, trans);
            
            // get wrist offset:
            // similar triangles 
         /*   Point3d armpos(-104, -70, 0);
            Point3d relpos = worldpt - armpos;
            double wristlen = 70;
            double d_armpos = sqrt(relpos.x * relpos.x + relpos.y * relpos.y);
            double dx = wristlen / (d_armpos / relpos.x);
            double dy = wristlen / (d_armpos / relpos.y);
            worldpt.x -= dx;
            worldpt.y -= dy;*/

            // slope
            //double slope = relpos.y / relpos.x;
            //worldpt.y -= 50 * slope;
            //worldpt.x -= 50 * 1/slope;

            circle(outframe, track.base, 5, Scalar(0, 255, 0), 2);
            putText(outframe,
                format("(%.3f, %.3f, %.3f)", worldpt.x, worldpt.y, worldpt.z),
                track.base,
                //clickpt,
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));


            int k = waitKey(10);
            if (k == 101) {
                std::cout << k << std::endl;
            
                int32_t x = worldpt.x, y = worldpt.y, z = worldpt.z;
                xn::send_int(cap.client_sock, x);
                xn::send_int(cap.client_sock, y);
                xn::send_int(cap.client_sock, z);


                Mat tv_oframe = trans.rotation * trans.tvec;
                tv_oframe *= -1;
                x = tv_oframe.at<double>(0);
                y = tv_oframe.at<double>(1);
                z = tv_oframe.at<double>(2);
                xn::send_int(cap.client_sock, x);
                xn::send_int(cap.client_sock, y);
                xn::send_int(cap.client_sock, z);
            }
            }

            for (Point2f c : corners) {
                circle(outframe, c, 5, Scalar(0, 0, 255));
            }
            circle(outframe, corners[0], 5, Scalar(255, 0, 0));
            //cout << corners[0] << endl;

            //for (Point2f c : cornersProjected) {
            //    circle(frame, c, 5, Scalar(255, 0, 0));
            //}

            //for (int i = 0; i < cornersProjected.size(); i++) {
            //    line(frame, corners[i], cornersProjected[i], Scalar(255, 0, 0), 2);
            //}

            //circle(frame, clickpt, 5, Scalar(0, 255, 0), 2);
            
            putText(outframe, cv::format("tvec = (%.2f, %.2f, %.2f)", trans.tvec.at<double>(0), trans.tvec.at<double>(1), trans.tvec.at<double>(2)), Point2i(25, 75), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0));
            Mat tvec_oframe = trans.rotation * trans.tvec;
            tvec_oframe *= -1;
            putText(outframe, cv::format("tvec_oframe = (%.2f, %.2f, %.2f)", tvec_oframe.at<double>(0), tvec_oframe.at<double>(1), tvec_oframe.at<double>(2)), Point2i(25, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0));


        }
        imshow(kWinName, outframe);

        // quit on escape
        if (waitKey(10) == 27)
            break;

    }
    return 0;
}

//int main(int argc, const char** argv) {
//    //camcalib::calibrate_camera("in_VID5.xml");
//    //return 0;
//    CommandLineParser parser(argc, argv, keys);
//    static const std::string kWinName = "CHARUCO";
//    namedWindow(kWinName);
//    xn::VideoStream cap(8080);
//
//    xn::homo::CharucoInfo charuco("out_camera_data.xml", 5, 7);
//
//    while (true) {
//        Mat frame, outFrame;
//        frame = cap.getLastFrame();
//        if (frame.empty())
//            continue;
//        frame.copyTo(outFrame);
//
//        xn::homo::detectCharucoBoard(frame, outFrame, charuco);
//
//        imshow("CHARUCO", outFrame);
//        if (waitKey(30) == 27)
//            break;
//    }
//
//    return 0;
//}

// // image classification (yolo dnn)
//int main(int argc, const char ** argv)
//{
//    CommandLineParser parser(argc, argv, keys);
//    Net net = xn::yolo::create_dnn(
//        parser.get<String>("model"), parser.get<String>("config"), parser.get<String>("classes"));
//
//    // blocks until a camera connects over TCP
//    xn::VideoStream vid(PORT);
//
//    // Create a window
//    static const std::string kWinName = "Deep learning object detection in OpenCV";
//    namedWindow(kWinName, WINDOW_NORMAL);
//    int initialConf = (int)(xn::yolo::confThreshold * 100);
//    createTrackbar("Confidence threshold, %", kWinName, &initialConf, 99, xn::yolo::setConfThreshold);
//
//    // Process frames.
//    Mat frame;
//    while (true)
//    {
//        // apply classifier to the frame
//        frame = vid.getLastFrame();
//        if (frame.rows > 0 && frame.cols > 0) {
//            xn::yolo::preprocess(frame, net);
//
//            std::vector<Mat> outs;
//            net.forward(outs, xn::yolo::outNames);
//
//            xn::yolo::postprocess(vid, frame, outs, net);
//
//            // Put efficiency information.
//            std::vector<double> layersTimes;
//            double freq = getTickFrequency() / 1000;
//            double t = net.getPerfProfile(layersTimes) / freq;
//            std::string label = format("Inference time: %.2f ms", t);
//            putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
//            imshow(kWinName, frame);
//        }
//
//        // quit on escape
//        if (waitKey(10) == 27)
//            break;
//    }
//    return 0;
//}
