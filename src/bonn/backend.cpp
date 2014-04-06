/*
 * backend.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: neek
 */



#include "bonn/backend.h"

namespace io {

	const std::string DataIOBonn::kPath = "/media/neek/DATA/MAINF2307/data/bonn";

	DataIOBonn::DataIOBonn() : seq_num_(1), frame_num_(0), seq_update_(true) {

		std::stringstream ss;
		ss << "labels.txt";
		std::string fname = "./" + ss.str();
		std::ifstream source;
		source.open(fname.c_str(), std::ios_base::in);

		for(std::string line; std::getline(source, line); )
		{
			std::istringstream in(line);

			for (int i = 0; i < 2*kLabelNum; i++) {
				int x, y;
				in >> x >> y;
				ground_truths_.push_back(x);
				ground_truths_.push_back(y);
			}
		}

	}
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

		// Ground truth
		int base_idx = 4*frame_num_*kLabelNum;
		ground_truth_ = std::vector<int>(kLabelNum*4);
		for (int i = 0; i < 4*kLabelNum; i++) {
			ground_truth_[i] = ground_truths_[base_idx + i];
		}

		// HEAD
		centre_truth_head.x = ground_truth_[6];
		centre_truth_head.y = ground_truth_[7];

		// LEFT HAND
		centre_truth_left_hand.x = ground_truth_[0];
		centre_truth_left_hand.y = ground_truth_[1];

		// RIGHT HAND
		centre_truth_right_hand.x = ground_truth_[8];
		centre_truth_right_hand.y = ground_truth_[9];

		// LEFT LEG
		centre_truth_left_foot.x = ground_truth_[12];
		centre_truth_left_foot.y = ground_truth_[13];

		// RIGHT LEG
		centre_truth_right_foot.x = ground_truth_[16];
		centre_truth_right_foot.y = ground_truth_[17];

		frame_num_++;
		return true;
	}

	void DataIOBonn::get_ground_truth(cv::Point& centre_head,
											cv::Point& centre_left_hand,
											cv::Point& centre_right_hand,
											cv::Point& centre_left_foot,
											cv::Point& centre_right_foot) {

		centre_head.x = to_x(centre_truth_head.x);
		centre_head.y = to_y(centre_truth_head.y);

		centre_left_hand.x = to_x(centre_truth_left_hand.x);
		centre_left_hand.y = to_y(centre_truth_left_hand.y);

		centre_right_hand.x = to_x(centre_truth_right_hand.x);
		centre_right_hand.y = to_y(centre_truth_right_hand.y);

		centre_left_foot.x = to_x(centre_truth_left_foot.x);
		centre_left_foot.y = to_y(centre_truth_left_foot.y);

		centre_right_foot.x = to_x(centre_truth_right_foot.x);
		centre_right_foot.y = to_y(centre_truth_right_foot.y);
	}

	void DataIOBonn::extract_parts() {
		/* initialize random seed: */
		srand (time(NULL));

		// Creating directory structure
		const std::string kTrainingDataPathHead = kPath + "/training";

		boost::filesystem::path training_data_path_head(kTrainingDataPathHead);
		boost::filesystem::create_directory(training_data_path_head);

		std::ofstream data((kTrainingDataPathHead + "/data.txt").c_str());


		if (!data.is_open()) {
			std::cerr << "Unable to create training data files!" << std::endl;
		}

		int head_idx, hand_idx, leg_idx;
		head_idx = hand_idx = leg_idx = 1;
		cv::Mat depth_img, rgb_img;
		while (read_next_frame(rgb_img, depth_img)) {

			assert(depth_img.type() == CV_32F);
			std::vector<int> positive_centres;

			// Head
			if (ground_truth_[6] > 0 && ground_truth_[4] > 0) {
				extract_positive_part(depth_img, data,
										ground_truth_[6], ground_truth_[7],
										ground_truth_[4], ground_truth_[5],
										classifier::HEAD);

				positive_centres.push_back(ground_truth_[6]);
				positive_centres.push_back(ground_truth_[7]);
			}

			// Left hand
			if (ground_truth_[0] > 0 && ground_truth_[2] > 0) {
				extract_positive_part(depth_img, data,
						ground_truth_[2], ground_truth_[3],
						ground_truth_[0], ground_truth_[1],
						classifier::LEFT_HAND);

				positive_centres.push_back(ground_truth_[0]);
				positive_centres.push_back(ground_truth_[1]);
			}

			// Right hand
			if (ground_truth_[8] > 0 && ground_truth_[10] > 0) {
				extract_positive_part(depth_img, data,
						ground_truth_[10], ground_truth_[11],
						ground_truth_[8], ground_truth_[9],
						classifier::RIGHT_HAND);

				positive_centres.push_back(ground_truth_[8]);
				positive_centres.push_back(ground_truth_[9]);
			}

			// Left leg
			if (ground_truth_[12] > 0 && ground_truth_[14] > 0) {
				extract_positive_part(depth_img, data,
						ground_truth_[14], ground_truth_[15],
						ground_truth_[12], ground_truth_[13],
						classifier::LEFT_FOOT);

				positive_centres.push_back(ground_truth_[12]);
				positive_centres.push_back(ground_truth_[13]);
			}

			// Right leg
			if (ground_truth_[16] > 0 && ground_truth_[17] > 0) {
				extract_positive_part(depth_img, data,
						ground_truth_[18], ground_truth_[19],
						ground_truth_[16], ground_truth_[17],
						classifier::RIGHT_FOOT);

				positive_centres.push_back(ground_truth_[16]);
				positive_centres.push_back(ground_truth_[17]);
			}

			for (int j = 0; j < 5; j++) {
				extract_negative_part(depth_img, data, positive_centres);
			}
		}
	}

	int DataIOBonn::to_x(float x_) { return x_; };
	int DataIOBonn::to_y(float y_) { return y_; };

}
