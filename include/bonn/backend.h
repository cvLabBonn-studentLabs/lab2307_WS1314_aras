/*
 * backend.h
 *
 *  Created on: Mar 27, 2014
 *      Author: neek
 */

#ifndef BACKEND_BONN_H_
#define BACKEND_BONN_H_

#include <iomanip>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include "data_io_backend.h"
#include "constants.h"

namespace io {

class DataIOBonn : public DataIOBackend {
public:
	DataIOBonn();
	DataIOBonn(int seq, int frame);
	~DataIOBonn();
	bool read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth);
	void get_ground_truth(cv::Point& centre_head,
											cv::Point& centre_left_hand,
											cv::Point& centre_right_hand,
											cv::Point& centre_left_foot,
											cv::Point& centre_right_foot);
	void extract_parts();
private:
	int to_x(float x_);
	int to_y(float y_);

	static const std::string kTrainingDataPath;

	int seq_num_;
	int frame_num_;
	bool seq_update_;
	std::vector<int> ground_truth_;
	static const std::string kPath;
	static const int kLabelNum = 5;
};

}


#endif /* BACKEND_H_ */
