#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <xn_net.hpp>

namespace xn 
{
	namespace homo
	{
		void createBoard(int width, int height) {
			cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(width, height, 0.04f, 0.02f, dictionary);
			cv::Mat boardImage;
			board->draw(cv::Size(600, 500), boardImage, 10, 1);
			cv::imwrite("charuco.png", boardImage);
		}

		static bool readCameraInfo(std::string filename, cv::Mat& cam_mat, cv::Mat& distortion_coeffs) {
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			if (!fs.isOpened())
				return false;
			fs["camera_matrix"] >> cam_mat;
			fs["distortion_coefficients"] >> distortion_coeffs;
			return true;
		}

		struct CharucoInfo {
			Mat cam_mat;
			Mat distortion;
			cv::Ptr<cv::aruco::Dictionary> dictionary;
			cv::Ptr<cv::aruco::CharucoBoard> board;
			cv::Ptr<cv::aruco::DetectorParameters> params;

			CharucoInfo(std::string cam_info_filename, int width, int height, float square_len = 28, float marker_len = 15) {
				if (!readCameraInfo(cam_info_filename, cam_mat, distortion)) {
					std::cerr << "Invalid camera file!\n";
					exit(-1);
				}

				dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
				board = cv::aruco::CharucoBoard::create(width, height, square_len, marker_len, dictionary);
				params = cv::aruco::DetectorParameters::create();
			}
		};

		void detectCharucoBoard(Mat& inputFrame, Mat& outputFrame, CharucoInfo& charuco) {
			std::vector<int> markerIds;
			std::vector<std::vector<cv::Point2f>> markerCorners, rejectedMarkers;

			cv::aruco::detectMarkers(inputFrame, charuco.board->dictionary, markerCorners, markerIds, charuco.params, rejectedMarkers);
			//for (int i =0; i < 10; i++ )
			cv::aruco::refineDetectedMarkers(inputFrame, charuco.board, markerCorners, markerIds, rejectedMarkers,
				charuco.cam_mat, charuco.distortion);

			if (rejectedMarkers.size() > 0)
				cv::aruco::drawDetectedMarkers(outputFrame, rejectedMarkers, noArray(), Scalar(100, 0, 255));

			if (markerIds.size() > 0) {
				cv::aruco::drawDetectedMarkers(outputFrame, markerCorners, markerIds);
				std::vector<cv::Point2f> charucoCorners;
				std::vector<int> charucoIds;

				cv::aruco::interpolateCornersCharuco(
					markerCorners, markerIds, inputFrame,
					charuco.board, charucoCorners, charucoIds, charuco.cam_mat, charuco.distortion);
				if (charucoIds.size() > 0) {
					cv::Scalar color = cv::Scalar(255, 0, 0);
					cv::aruco::drawDetectedCornersCharuco(outputFrame, charucoCorners, charucoIds, color);
					cv::Vec3d rvec, tvec;
					bool valid = cv::aruco::estimatePoseCharucoBoard(
						charucoCorners, charucoIds, charuco.board, charuco.cam_mat, charuco.distortion, rvec, tvec);
					// if charuco pose is valid
					if (valid)
						cv::aruco::drawAxis(outputFrame, charuco.cam_mat, charuco.distortion, rvec, tvec, 50);
				}
			}
		}

		struct HomoTransform {
			cv::Mat tvec;
			cv::Mat rotation;
		};

		void GetHomoTransform(Mat& H, HomoTransform& out) {
			// Normalization to ensure that ||c1|| = 1
			double norm = sqrt(
				H.at<double>(0, 0) * H.at<double>(0, 0) +
				H.at<double>(1, 0) * H.at<double>(1, 0) +
				H.at<double>(2, 0) * H.at<double>(2, 0));
			H /= norm;
			Mat c1 = H.col(0);
			Mat c2 = H.col(1);
			Mat c3 = c1.cross(c2);
			Mat tvec = H.col(2);
			Mat R(3, 3, CV_64F);
			for (int i = 0; i < 3; i++)
			{
				R.at<double>(i, 0) = c1.at<double>(i, 0);
				R.at<double>(i, 1) = c2.at<double>(i, 0);
				R.at<double>(i, 2) = c3.at<double>(i, 0);
			}

			Mat W, U, Vt;
			SVDecomp(R, W, U, Vt);
			out.rotation = U * Vt;
			out.tvec = tvec;
		}

		void inverseProjectPoint(const Point2i& uv, Point3d& out, const Mat& cam_mat, const HomoTransform& transform) {
			Mat muv = (Mat_<double>(3, 1) << uv.x, uv.y, 1);

			Mat ls = transform.rotation.inv() * cam_mat.inv() * muv;
			Mat rs = transform.rotation.inv() * transform.tvec;

			double s = rs.at<double>(2, 0) / ls.at<double>(2, 0);
			Mat temp = transform.rotation.inv() * (s * cam_mat.inv() * muv - transform.tvec);
			out = Point3d(temp.at<double>(0), temp.at<double>(1), temp.at<double>(2));
		}

		std::vector<cv::Point3d> Face3D({
			cv::Point3d(6.825897, 6.760612, 4.402142),
			cv::Point3d(1.330353, 7.122144, 6.903745),     //#29 left brow right corner
			cv::Point3d(-1.330353, 7.122144, 6.903745),    //#34 right brow left corner
			cv::Point3d(-6.825897, 6.760612, 4.402142),    //#38 right brow right corner
			cv::Point3d(5.311432, 5.485328, 3.987654),     //#13 left eye left corner
			cv::Point3d(1.789930, 5.393625, 4.413414),     //#17 left eye right corner
			cv::Point3d(-1.789930, 5.393625, 4.413414),    //#25 right eye left corner
			cv::Point3d(-5.311432, 5.485328, 3.987654),    //#21 right eye right corner
			cv::Point3d(2.005628, 1.409845, 6.165652),     //#55 nose left corner
			cv::Point3d(-2.005628, 1.409845, 6.165652),    //#49 nose right corner
			cv::Point3d(2.774015, -2.080775, 5.048531),    //#43 mouth left corner
			cv::Point3d(-2.774015, -2.080775, 5.048531),   //#39 mouth right corner
			cv::Point3d(0.000000, -3.116408, 6.097667),    //#45 mouth central bottom corner
			cv::Point3d(0.000000, -7.415691, 4.070434)    //#6 chin corner
			});

		std::vector<cv::Point3d> facecube_src ({
			cv::Point3d(10.0, 10.0, 10.0),
			cv::Point3d(10.0, 10.0, -10.0),
			cv::Point3d(10.0, -10.0, -10.0),
			cv::Point3d(10.0, -10.0, 10.0),
			cv::Point3d(-10.0, 10.0, 10.0),
			cv::Point3d(-10.0, 10.0, -10.0),
			cv::Point3d(-10.0, -10.0, -10.0),
			cv::Point3d(-10.0, -10.0, 10.0),
			cv::Point3d(0, 0, 0)
		});


	}
}