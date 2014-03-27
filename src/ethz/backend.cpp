/*
 * backend.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: neek
 */

#include "ethz/backend.h"

namespace io {

const std::string DataIOETHZ::kDatasetPath = "data/ethz";

DataIOETHZ::DataIOETHZ()
	: object_id_(0),
	  session_id_(0),
	  frame_id_(1),
	  update_session_(true) {}


bool DataIOETHZ::read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth) {
	std::stringstream ss;
	ss << "table-object" << std::setfill('0') << std::setw(2) << object_id_;

	std::string data_dir = kDatasetPath + "/" + ss.str();
	if (!boost::filesystem::is_directory(data_dir)) {
		object_id_++;
		session_id_ = 0;
		frame_id_ = 1;
		if (object_id_ > 17) {
			return false;
		} else {
			update_session_ = true;
			return read_next_frame(frame_rgb, frame_depth);
		}
	}

	ss << "/" << "session" << std::setfill('0') << std::setw(3) << session_id_
		<< "_depth_" << std::setfill('0') << std::setw(5) << frame_id_ << ".png";

	std::string filename = kDatasetPath + "/" + ss.str();
	if (!boost::filesystem::exists(filename)) {
		if (frame_id_ < 100) {
			frame_id_++;
		} else if (session_id_ < 30) {
			session_id_++;
			frame_id_ = 1;
		} else {
			object_id_++;
			session_id_ = 0;
			frame_id_ = 1;
		}
		update_session_ = true;
		return read_next_frame(frame_rgb, frame_depth);
	}

	std::cout << "Reading object " << object_id_ << " session " << session_id_ << " frame " << frame_id_ << std::endl;
	load_float_image(filename, frame_depth);

	if (update_session_) {
		read_ground_truth();
		update_session_ = false;
	}

	float x_, y_, z_, x, y, z;
	int base_idx = frame_id_*kLabelNum*3;
	x_ = ground_truth_[base_idx];
	y_ = ground_truth_[base_idx + 1];
	z_ = ground_truth_[base_idx + 1];

	x = to_x(x_, y_, z_);
	y = to_y(x_, y_, z_);

	std::cout << "x_ = " << x_ << " y_ = " << y_ << " z_ " << z_ << std::endl;
	std::cout << "x = " << x << " y = " << y << std::endl;

	std::cout << frame_depth.size() << std::endl;

	cv::Mat display;
	cv::normalize(frame_depth, display, 0.0, 255.0, cv::NORM_MINMAX);
	display.convertTo(display, CV_8UC1);

	cv::circle(display,
			cv::Point(x, y),
			 1.0,
			 cv::Scalar( 255, 255, 255 ),
			 -1,
			 8);

	cv::imshow("Testing image", display);
	cv::waitKey(0);

/*
	assert(frame_depth.cols == 640 && frame_depth.rows == 480);

	cv::Mat roi(frame, cv::Rect(140, 20, 360, 440));
	cv::resize(roi, frame_depth, cv::Size(kInputWidth, kInputHeight));
	cv::cvtColor(frame_depth, frame_depth, CV_RGB2GRAY);
	frame_depth.convertTo(frame_depth, CV_32F);
	cv::normalize(frame_depth, frame_depth, 0.0, 1.0, cv::NORM_MINMAX);

	for(int row = 0; row < frame_depth.rows; ++row) {
		float* p = frame_depth.ptr<float>(row);
	    for(int col = 0; col < frame_depth.cols; ++col) {
	    	if (*p < 0.01f) {
	    		*p = 0.2f;
	    	} else {
	    		*p = 1.0f - *p;

	    		if (*p > 0.9f) {
	    			*p = 0.8f;
	    		}
	    	}
	    	p++;
	    }
	}*/

	frame_id_++;
	return true;
}


int DataIOETHZ::to_x(float x_, float y_, float z_) { return static_cast<int>(round(176.f/2.f - x_ - 15.f)); }
int DataIOETHZ::to_y(float x_, float y_, float z_) { return static_cast<int>(round(144.f/2.f - y_ - 200.f)); }

void DataIOETHZ::read_ground_truth() {
	std::stringstream ss;
	ss << "table-object" << std::setfill('0') << std::setw(2) << object_id_
			<< "/out" << std::setfill('0') << std::setw(3) << session_id_ << "_1";

	std::string data_dir = kDatasetPath + "/" + ss.str();
	if (!boost::filesystem::is_directory(data_dir)) {
		std::cerr << "No ground truth file found in: " << data_dir << std::endl;
		return;
	}

	ss << "/absXYZ.txt";
	std::string filename = kDatasetPath + "/" + ss.str();
	if (!boost::filesystem::exists(filename)) {
		std::cerr << "No ground truth file found: " << filename << std::endl;
		return;
	}

	std::ifstream source;
	source.open(filename.c_str(), std::ios_base::in);

	for(std::string line; std::getline(source, line); )
	{
	    std::istringstream in(line);

	    for (int i = 0; i < kLabelNum; i++) {
	    	float x, y, z;
	    	in >> x >> y >> z;
	    	ground_truth_.push_back(x);
	    	ground_truth_.push_back(y);
	    	ground_truth_.push_back(z);
	    }
	}
}


}
