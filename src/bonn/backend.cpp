/*
 * backend.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: neek
 */



#include "bonn/backend.h"

namespace io {

	const std::string DataIOBonn::kPath = "/media/neek/DATA/MAINF2307/data/bonn";

	DataIOBonn::DataIOBonn() : seq_num_(1), frame_num_(0), seq_update_(true) {}
	DataIOBonn::DataIOBonn(int seq, int frame) : seq_num_(seq), frame_num_(frame) {}
	DataIOBonn::~DataIOBonn() {}

	bool DataIOBonn::read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth) {

		std::stringstream ss;
		ss << seq_num_;

		std::string data_dir = kPath + "/" + ss.str();

		if (!boost::filesystem::is_directory(data_dir))
			return false;

		ss << "/" << std::setfill('0') << std::setw(3) << frame_num_ << ".png";
		std::string filename = kPath + "/" + ss.str();

		if (!boost::filesystem::exists(filename)) {
			seq_num_++;
			frame_num_ = 0;
			seq_update_ = true;
			return read_next_frame(frame_rgb, frame_depth);
		}

		std::cout << "Reading sequence " << seq_num_ << " frame " << frame_num_ << std::endl;
		cv::Mat frame = cv::imread(filename);

		assert(frame.cols == 640 && frame.rows == 480);

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
		}

		if (seq_update_) {
			std::stringstream ss;
			ss << seq_num_ << ".txt";
			std::string fname = kPath + "/" + ss.str();
			std::ifstream source;
			source.open(fname.c_str(), std::ios_base::in);

			for(std::string line; std::getline(source, line); )
			{
			    std::istringstream in(line);

			    for (int i = 0; i < kLabelNum; i++) {
			    	int x, y, width, height;
			    	in >> x >> y >> width >> height;
			    	ground_truth_.push_back(x);
			    	ground_truth_.push_back(y);
			    }
			}
			seq_update_ = false;
		}

		// Ground truth
		int base_idx = 2*frame_num_*kLabelNum;

		// HEAD
		centre_truth_head.x = ground_truth_[base_idx];
		centre_truth_head.y = ground_truth_[base_idx + 1];

		// LEFT HAND
		centre_truth_left_hand.x = to_x(ground_truth_[base_idx + 2]);
		centre_truth_left_hand.y = to_y(ground_truth_[base_idx + 3]);

		// RIGHT HAND
		centre_truth_right_hand.x = to_x(ground_truth_[base_idx + 4]);
		centre_truth_right_hand.y = to_y(ground_truth_[base_idx + 5]);

		// LEFT LEG
		centre_truth_left_foot.x = to_x(ground_truth_[base_idx + 6]);
		centre_truth_left_foot.y = to_y(ground_truth_[base_idx + 7]);

		// RIGHT LEG
		centre_truth_right_foot.x = to_x(ground_truth_[base_idx + 8]);
		centre_truth_right_foot.y = to_y(ground_truth_[base_idx + 9]);


//		int x = to_x(centre_truth_head.x);
//		int y = to_y(centre_truth_head.y);
//
//		cv::Mat display;
//		cv::normalize(frame_depth, display, 0.0, 255.0, cv::NORM_MINMAX);
//		display.convertTo(display, CV_8UC1);
//
//		cv::circle(display,
//				cv::Point(x, y),
//				 3.0,
//				 cv::Scalar( 255, 255, 255 ),
//				 -1,
//				 8);
//
//		cv::imshow("Testing image", display);
//		cv::waitKey(0);

		frame_num_++;
		return true;
	}

	void DataIOBonn::get_ground_truth(cv::Point& centre_head,
											cv::Point& centre_left_hand,
											cv::Point& centre_right_hand,
											cv::Point& centre_left_foot,
											cv::Point& centre_right_foot) {
		centre_head.x = centre_truth_head.x;
		centre_head.y = centre_truth_head.y;

		centre_left_hand.x = centre_truth_left_hand.x;
		centre_left_hand.y = centre_truth_left_hand.y;

		centre_right_hand.x = centre_truth_right_hand.x;
		centre_right_hand.y = centre_truth_right_hand.y;

		centre_left_foot.x = centre_truth_left_foot.x;
		centre_left_foot.y = centre_truth_left_foot.y;

		centre_right_foot.x = centre_truth_right_foot.x;
		centre_right_foot.y = centre_truth_right_foot.y;
	}

	void DataIOBonn::extract_parts() {
		/* initialize random seed: */
		srand (time(NULL));

		// Creating directory structure
		const std::string kTrainingDataPathHead = kPath + "training/";

		boost::filesystem::path training_data_path_head(kTrainingDataPathHead);
		boost::filesystem::create_directory(training_data_path_head);

		std::ofstream data((kTrainingDataPathHead + "data.txt").c_str());


		if (!data.is_open()) {
			std::cerr << "Unable to create training data files!" << std::endl;
		}

		int head_idx, hand_idx, leg_idx;
		head_idx = hand_idx = leg_idx = 1;
		cv::Mat depth_img, rgb_img, frame;
		while (read_next_frame(rgb_img, depth_img)) {
			std::vector<int> positive_centres;

			cv::normalize(depth_img, frame, 0, 255, cv::NORM_MINMAX);
			frame.convertTo(frame, CV_8UC1);

			// Head
			extract_positive_part(frame, data,
									centre_truth_head.x, centre_truth_head.y + 15,
									centre_truth_head.x, centre_truth_head.y + 13,
									classifier::HEAD);

//			positive_centres.push_back(f.markers[3].x);
//			positive_centres.push_back(f.markers[3].y);
//
//			// Left hand
//			if (f.markers[28].cond > 0 && f.markers[24].cond > 0) {
//				extract_positive_part(frame, head_data,
//										f.markers[28].x, f.markers[28].y,
//										f.markers[24].x, f.markers[24].y,
//										classifier::LEFT_HAND);
//
//				positive_centres.push_back(f.markers[28].x);
//				positive_centres.push_back(f.markers[28].y);
//			}
//
//			// Right hand
//			if (f.markers[19].cond > 0 && f.markers[16].cond > 0) {
//				extract_positive_part(frame, head_data,
//										f.markers[19].x, f.markers[19].y,
//										f.markers[16].x, f.markers[16].y,
//										classifier::RIGHT_HAND);
//
//				positive_centres.push_back(f.markers[19].x);
//				positive_centres.push_back(f.markers[19].y);
//			}
//
//			// Left leg
//			/*if (f.markers[35].cond > 0 && f.markers[37].cond > 0) {
//				extract_positive_part(frame, head_data,
//										f.markers[35].x, f.markers[35].y,
//										f.markers[37].x, f.markers[37].y,
//										classifier::LEFT_FOOT);
//
//				positive_centres.push_back(f.markers[35].x);
//				positive_centres.push_back(f.markers[35].y);
//			}
//
//			// Right leg
//			if (f.markers[43].cond > 0 && f.markers[41].cond > 0) {
//				extract_positive_part(frame, head_data,
//										f.markers[43].x, f.markers[43].y,
//										f.markers[41].x, f.markers[41].y,
//										classifier::RIGHT_FOOT);
//
//				positive_centres.push_back(f.markers[43].x);
//				positive_centres.push_back(f.markers[43].y);
//			}*/
//
//			extract_negative_part(frame, head_data, positive_centres);
		}
	}

	int DataIOBonn::to_x(float x_) { return (x_ - 140.f)/2.5f; };
	int DataIOBonn::to_y(float y_) { return (y_ - 20.f)/2.5f; };

}
