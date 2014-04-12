/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Computer Science Institute III, University of Bonn
 *  Author: Nikita Araslanov, 18.03.2014
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

#ifndef DATA_IO_BACKEND_H_
#define DATA_IO_BACKEND_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "constants.h"
#include "classifier_svm.h"
#include "keyframe.h"

namespace io {

class DataIOBackend {
public:
	DataIOBackend();
	virtual ~DataIOBackend();

	virtual bool read_next_frame(cv::Mat& frame_rgb, cv::Mat& depth_rgb);
	virtual void extract_parts();
	virtual void get_ground_truth(cv::Point& centre_head,
									cv::Point& centre_left_hand,
									cv::Point& centre_right_hand,
									cv::Point& centre_left_foot,
									cv::Point& centre_right_foot);
	virtual float getScaleZ() { return 1.f; };

public:
	static float intersection_ratio(cv::Point topleft1, cv::Point topleft2) {
		float isc_x = std::max(std::min(topleft1.x,	topleft2.x) + pose::kDescriptorSize
								- std::max(topleft1.x, topleft2.x), 0);

		float isc_y = std::max(std::min(topleft1.y, topleft2.y) + pose::kDescriptorSize
								- std::max(topleft1.y, topleft2.y), 0);

		return isc_x*isc_y/static_cast<float>(pose::kDescriptorSize*pose::kDescriptorSize);
	}

protected:
	bool load_float_image(std::string filename, cv::Mat& output);
	void image_to_string(cv::Mat image, std::stringstream& line);

	void extract_positive_part(cv::Mat& frame,
								std::ofstream& ostream,
								float x1_, float y1_,
								float x2_, float y2_,
								classifier::BodyPart id);
	void extract_positive_part(cv::Mat& frame,
								std::ofstream& ostream,
								float x1_, float y1_,
								float x2_, float y2_,
								classifier::BodyPart id,
								bool mirror);
	void extract_negative_part(cv::Mat& frame,
								std::ofstream& ostream,
								std::vector<int> positives);

	virtual int to_x(float x_) { return x_; };
	virtual int to_y(float y_) { return y_; };

	cv::Point centre_truth_head;
	cv::Point centre_truth_left_hand;
	cv::Point centre_truth_right_hand;
	cv::Point centre_truth_left_foot;
	cv::Point centre_truth_right_foot;

private:


};

}

#endif /* DATA_IO_BACKEND_H_ */
