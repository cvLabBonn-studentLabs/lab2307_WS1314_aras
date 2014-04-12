/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Computer Science Institute III, University of Bonn
 *  Author: Nikita Araslanov, 27.03.2014
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     III nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
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

	float getScaleZ() { return 200.f; };
private:
	int to_x(float x_);
	int to_y(float y_);

	static const std::string kTrainingDataPath;

	int seq_num_;
	int frame_num_;
	bool seq_update_;
	std::vector<int> ground_truth_;
	std::vector<int> ground_truths_;
	static const std::string kPath;
	static const int kLabelNum = 5;
};

}


#endif /* BACKEND_H_ */
