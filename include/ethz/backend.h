/*
 * backend.h
 *
 *  Created on: Mar 18, 2014
 *      Author: neek
 */

#ifndef BACKEND_ETHZ_H_
#define BACKEND_ETHZ_H_

#include <iomanip>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include "data_io_backend.h"
#include "constants.h"

namespace io {

class DataIOETHZ : public DataIOBackend {
public:
	DataIOETHZ();
	bool read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth);
private:
	int object_id_;
	int session_id_;
	int frame_id_;
	bool update_session_;
	std::vector<float> ground_truth_;

	static const int kLabelNum = 13;
	static const std::string kDatasetPath;
	void read_ground_truth();
	int to_x(float x_, float y_, float z_);
	int to_y(float x_, float y_, float z_);
};

}

#endif /* BACKEND_H_ */
