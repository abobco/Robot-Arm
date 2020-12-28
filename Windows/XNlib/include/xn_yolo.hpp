#pragma once
#include <xn_net.hpp>
#include <fstream>

namespace xn {
	namespace yolo {

		struct TrackingInfo 
		{
			std::vector<std::string> classes;
			std::string label;
			int id;
			Point2i center;
			Point2i base;
			bool in_frame = false;

			TrackingInfo(std::string _label, std::string classfile) {
				label = _label;

				// load classfile
				this->classes = std::vector<std::string>();
				std::ifstream ifs(classfile.c_str());
				if (!ifs.is_open())
					CV_Error(Error::StsError, "File " + classfile + " not found");
				std::string line;
				while (std::getline(ifs, line))
					this->classes.push_back(line);

				// get int idx of target object string
				auto it = std::find(this->classes.begin(), this->classes.end(), label);
				if (it == this->classes.end()) {
					printf("Label, '%s' not found in classfile!\n", label.c_str());
					throw -1;
				}
				id = it - this->classes.begin();
				center = Point2i(0, 0);
				base = Point2i(0, 0);
			}
		};

		struct YoloInfo 
		{
			Size inp_size;
			float scale;
			float conf_thres;
			float nms_thres;
			bool swap_rb;
			std::vector<std::string> out_names;
			
			int backend;
			/*
				0: automatically (by default)
				1: Halide language (http://halide-lang.org/)
				2: Intel's Deep Learning Inference Engine (https://software.intel.com/openvino-toolkit)
				3: OpenCV implementation
			*/

			int device;
			/*
				0: CPU target (by default)
				1: OpenCL
				2: OpenCL fp16 (half-float precision)
				3: VPU
			*/

			YoloInfo(Size _inp_size, float _conf_thres=0.5, float _nms_thres=0.4, float _scale=0.00392f, bool _swap_rb=false, int _backend=0, int _device=0) {
				inp_size = _inp_size;
				scale = _scale;
				swap_rb = _swap_rb;
				conf_thres = _conf_thres;
				nms_thres = _nms_thres;
				backend = _backend;
				device = _device;
				out_names = std::vector<std::string>();
			}
		};

		void create_dnn(YoloInfo& yo, dnn::Net& out_net, std::string modelpath, std::string configpath) {
			dnn::Net net = dnn::readNet(modelpath, configpath);
			net.setPreferableBackend(yo.backend);
			net.setPreferableTarget(yo.device);
			yo.out_names = net.getUnconnectedOutLayersNames();
			out_net = net;
		}

		inline void preprocess(const cv::Mat& frame, YoloInfo& yo, dnn::Net& net) {
			static Mat blob;

			// Create a 4D blob from a frame.
			if (yo.inp_size.width <= 0) yo.inp_size.width = frame.cols;
			if (yo.inp_size.height <= 0) yo.inp_size.height = frame.rows;
			dnn::blobFromImage(frame, blob, 1.0, yo.inp_size, Scalar(), yo.swap_rb, false, CV_8U);

			// Run a model.
			net.setInput(blob, "", yo.scale);
			if (net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
			{
				resize(frame, frame, yo.inp_size);
				Mat imInfo = (Mat_<float>(1, 3) << yo.inp_size.height, yo.inp_size.width, 1.6f);
				net.setInput(imInfo, "im_info");
			}
		}

		bool drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame, TrackingInfo& track);

		void postprocess(Mat& frame, const std::vector<Mat>& outs, dnn::Net& net, YoloInfo& yo, TrackingInfo& track)
		{
			static std::vector<int> outLayers = net.getUnconnectedOutLayers();
			static std::string outLayerType = net.getLayer(outLayers[0])->type;

			std::vector<int> classIds;
			std::vector<float> confidences;
			std::vector<Rect> boxes;
			if (outLayerType == "DetectionOutput")
			{
				// Network produces output blob with a shape 1x1xNx7 where N is a number of
				// detections and an every detection is a vector of values
				// [batchId, classId, confidence, left, top, right, bottom]
				CV_Assert(outs.size() > 0);
				for (size_t k = 0; k < outs.size(); k++)
				{
					float* data = (float*)outs[k].data;
					for (size_t i = 0; i < outs[k].total(); i += 7)
					{
						float confidence = data[i + 2];
						if (confidence > yo.conf_thres)
						{
							int left = (int)data[i + 3];
							int top = (int)data[i + 4];
							int right = (int)data[i + 5];
							int bottom = (int)data[i + 6];
							int width = right - left + 1;
							int height = bottom - top + 1;
							if (width <= 2 || height <= 2)
							{
								left = (int)(data[i + 3] * frame.cols);
								top = (int)(data[i + 4] * frame.rows);
								right = (int)(data[i + 5] * frame.cols);
								bottom = (int)(data[i + 6] * frame.rows);
								width = right - left + 1;
								height = bottom - top + 1;
							}
							classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
							boxes.push_back(Rect(left, top, width, height));
							confidences.push_back(confidence);
						}
					}
				}
			}
			else if (outLayerType == "Region")
			{
				for (size_t i = 0; i < outs.size(); ++i)
				{
					// Network produces output blob with a shape NxC where N is a number of
					// detected objects and C is a number of classes + 4 where the first 4
					// numbers are [center_x, center_y, width, height]
					float* data = (float*)outs[i].data;
					for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
					{
						Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
						Point classIdPoint;
						double confidence;
						minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
						if (confidence > yo.conf_thres)
						{
							int centerX = (int)(data[0] * frame.cols);
							int centerY = (int)(data[1] * frame.rows);
							int width = (int)(data[2] * frame.cols);
							int height = (int)(data[3] * frame.rows);
							int left = centerX - width / 2;
							int top = centerY - height / 2;

							classIds.push_back(classIdPoint.x);
							confidences.push_back((float)confidence);
							boxes.push_back(Rect(left, top, width, height));
						}
					}
				}
			}
			else
				CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);

			// NMS is used inside Region layer only on DNN_BACKEND_OPENCV for another backends we need NMS in sample
			// or NMS is required if number of outputs > 1
			if (outLayers.size() > 1 || (outLayerType == "Region" && yo.backend != dnn::DNN_BACKEND_OPENCV))
			{
				std::map<int, std::vector<size_t> > class2indices;
				for (size_t i = 0; i < classIds.size(); i++)
				{
					if (confidences[i] >= yo.conf_thres)
					{
						class2indices[classIds[i]].push_back(i);
					}
				}
				std::vector<Rect> nmsBoxes;
				std::vector<float> nmsConfidences;
				std::vector<int> nmsClassIds;
				for (std::map<int, std::vector<size_t> >::iterator it = class2indices.begin(); it != class2indices.end(); ++it)
				{
					std::vector<Rect> localBoxes;
					std::vector<float> localConfidences;
					std::vector<size_t> classIndices = it->second;
					for (size_t i = 0; i < classIndices.size(); i++)
					{
						localBoxes.push_back(boxes[classIndices[i]]);
						localConfidences.push_back(confidences[classIndices[i]]);
					}
					std::vector<int> nmsIndices;
					dnn::NMSBoxes(localBoxes, localConfidences, yo.conf_thres, yo.nms_thres, nmsIndices);
					for (size_t i = 0; i < nmsIndices.size(); i++)
					{
						size_t idx = nmsIndices[i];
						nmsBoxes.push_back(localBoxes[idx]);
						nmsConfidences.push_back(localConfidences[idx]);
						nmsClassIds.push_back(it->first);
					}
				}
				boxes = nmsBoxes;
				classIds = nmsClassIds;
				confidences = nmsConfidences;
			}

			track.in_frame = false;
			for (size_t idx = 0; idx < boxes.size(); ++idx)
			{
				Rect box = boxes[idx];
				bool res = drawPred(classIds[idx], confidences[idx], box.x, box.y,
					box.x + box.width, box.y + box.height, frame, track);
				if (res) track.in_frame = true;
			}
			//if (target_in_frame) {
			//	//std::string label = format("(%d, %d)", target_center[0], target_center[1]);
			//	//putText(frame, label, Point(20, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
	  //      
			//	xn::send_int(vid.client_sock, target_center[0]);
			//	xn::send_int(vid.client_sock, target_center[1]);
			//} else {
			//	xn::send_int(vid.client_sock, 0);
			//	xn::send_int(vid.client_sock, 0);
			//}

		}

		bool drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame, TrackingInfo& track)
		{
			bool ret = false;
			Scalar rect_color(0, 255, 0);
	    
			std::string label = format("%.2f", conf);
			if (!track.classes.empty())
			{
				CV_Assert(classId < (int)track.classes.size());
				label = track.classes[classId] + ": " + label;
				if (classId == track.id) {
					// putText(frame, "BINGO", Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
					track.center = Point2i((left + right) / 2, (top + bottom) / 2);
					track.base = Point2i((left + right) / 2, bottom);
					ret = true;
					rect_color = Scalar(0, 0, 255);
				}
			}
	
			rectangle(frame, Point(left, top), Point(right, bottom), rect_color);
	  
			int baseLine;
			Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	
			top = max(top, labelSize.height);
			rectangle(frame, Point(left, top - labelSize.height),
				Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
			putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
			return ret;
		}
	
		//void setConfThreshold(int pos, void*)
		//{
		//	confThreshold = pos * 0.01f;
		//}
	}
}