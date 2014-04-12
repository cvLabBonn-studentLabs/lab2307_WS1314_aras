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

#include <boost/filesystem.hpp>
#include "keyframe.h"
#include "classifier_svm.h"
#include "stanford/backend.h"
#include "constants.h"
#include "read_data_eval.cpp"

namespace io {

const std::string DataIOStanford::kPath = "data/stanford";

DataIOStanford::DataIOStanford() { stanford_eval::id = 0; }


bool DataIOStanford::read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth) {
//	while (true) {
		if(!stanford_eval::read_frame()) {  if (!stanford_eval::next_file()) return false; };
		stanford_eval::Frame f = stanford_eval::frame;

		cv::Mat frame(cv::Size(f.C, f.R), CV_32F);

		for (int i = 0; i < f.N; ++i) {
			frame.at<float>(i % f.R, i/f.R) = -(f.points[i][2] + 2.0);
		}

		cv::normalize(frame, frame_depth, 0, 1.0, cv::NORM_MINMAX);

		for (int i = 0; i < f.N; ++i) {
			if (frame_depth.at<float>(i % f.R, i/f.R) > 0.7f)
				frame_depth.at<float>(i % f.R, i/f.R) = 0.0f;
		}

		// Ground truth
		if (f.markers[1].cond > 0) {
			centre_truth_head.x = to_x(f.markers[1].x);
			centre_truth_head.y = to_y(f.markers[1].y + 0.1);
		} else if (f.markers[3].cond > 0) {
			centre_truth_head.x = to_x(f.markers[3].x);
			centre_truth_head.y = to_y(f.markers[3].y + 0.1);
		} else {
			centre_truth_head.x = -1;
		}

		if (f.markers[24].cond > 0) {
			centre_truth_left_hand.x = to_x(f.markers[24].x);
			centre_truth_left_hand.y = to_y(f.markers[24].y);
		} else if (f.markers[28].cond > 0) {
			centre_truth_left_hand.x = to_x(f.markers[28].x);
			centre_truth_left_hand.y = to_y(f.markers[28].y);
		} else {
			centre_truth_left_hand.x = -1;
		}

		if (f.markers[16].cond > 0) {
			centre_truth_right_hand.x = to_x(f.markers[16].x);
			centre_truth_right_hand.y = to_y(f.markers[16].y);
		} else if (f.markers[19].cond > 0) {
			centre_truth_right_hand.x = to_x(f.markers[19].x);
			centre_truth_right_hand.y = to_y(f.markers[19].y);
		} else {
			centre_truth_right_hand.x = -1;
		}

		if (f.markers[35].cond > 0) {
			centre_truth_left_foot.x = to_x(f.markers[35].x);
			centre_truth_left_foot.y = to_y(f.markers[35].y - 0.2);
		} else if (f.markers[37].cond > 0) {
			centre_truth_left_foot.x = to_x(f.markers[37].x);
			centre_truth_left_foot.y = to_y(f.markers[37].y - 0.2);
		} else {
			centre_truth_left_foot.x = -1;
		}

		if (f.markers[43].cond > 0) {
			centre_truth_right_foot.x = to_x(f.markers[43].x);
			centre_truth_right_foot.y = to_y(f.markers[43].y - 0.2);
		} else if (f.markers[41].cond > 0) {
			centre_truth_right_foot.x = to_x(f.markers[41].x);
			centre_truth_right_foot.y = to_y(f.markers[41].y - 0.2);
		} else {
			centre_truth_right_foot.x = -1;
		}

		return true;
}

void DataIOStanford::get_ground_truth(cv::Point& centre_head,
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

void DataIOStanford::extract_parts() {
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
	cv::Mat frame, frame_dummy;
	while (read_next_frame(frame_dummy, frame)) {
		//if(!) {  stanford_eval::next_file(); };
		stanford_eval::Frame f = stanford_eval::frame;

//		cv::Mat frame(cv::Size(f.C, f.R), CV_32F);
//
//		for (int i = 0; i < f.N; ++i) {
//			frame.at<float>(i % f.R, i/f.R) = -(static_cast<float>(f.points[i][2]) + 2.f);
//		}
//
//		cv::normalize(frame, frame, 0, 1.f, cv::NORM_MINMAX);
		std::vector<int> positive_centres;

		// Head
		if (f.markers[3].cond > 0 && f.markers[1].cond > 0) {
			extract_positive_part(frame, data,
									f.markers[3].x, f.markers[3].y + 0.1,
									f.markers[1].x, f.markers[1].y + 0.1,
									classifier::HEAD, true);

			extract_positive_part(frame, data,
									f.markers[3].x, f.markers[3].y + 0.1,
									f.markers[3].x, f.markers[1].y + 0.1,
									classifier::HEAD, true);

			positive_centres.push_back(to_x(f.markers[1].x));
			positive_centres.push_back(to_y(f.markers[1].y + 0.1));
			positive_centres.push_back(to_x(f.markers[3].x));
			positive_centres.push_back(to_y(f.markers[3].y + 0.1));
		}

		// Left hand
		if (f.markers[28].cond > 0 && f.markers[24].cond > 0) {
			extract_positive_part(frame, data,
									f.markers[28].x, f.markers[28].y,
									f.markers[24].x, f.markers[24].y,
									classifier::LEFT_HAND, true);

			positive_centres.push_back(to_x(f.markers[24].x));
			positive_centres.push_back(to_y(f.markers[24].y));
		}

		// Right hand
		if (f.markers[19].cond > 0 && f.markers[16].cond > 0) {
			extract_positive_part(frame, data,
									f.markers[19].x, f.markers[19].y,
									f.markers[16].x, f.markers[16].y,
									classifier::RIGHT_HAND, true);

			positive_centres.push_back(to_x(f.markers[16].x));
			positive_centres.push_back(to_y(f.markers[16].y));
		}

		// Left foot
		if (f.markers[35].cond > 0 && f.markers[37].cond > 0) {
			extract_positive_part(frame, data,
									f.markers[37].x, f.markers[37].y - 0.2,
									f.markers[35].x, f.markers[35].y - 0.2,
									classifier::LEFT_FOOT, true);

			positive_centres.push_back(to_x(f.markers[35].x));
			positive_centres.push_back(to_y(f.markers[35].y - 0.2));
		}

		// Right foot
		if (f.markers[43].cond > 0 && f.markers[41].cond > 0) {
			extract_positive_part(frame, data,
									f.markers[41].x, f.markers[41].y - 0.2,
									f.markers[43].x, f.markers[43].y - 0.2,
									classifier::RIGHT_FOOT, true);

			positive_centres.push_back(to_x(f.markers[43].x));
			positive_centres.push_back(to_y(f.markers[43].y - 0.2));
		}

		for (int j = 0; j < 12; j++) {
			extract_negative_part(frame, data, positive_centres);
		}
	}
}

int DataIOStanford::to_x(float x_) { return std::min(144, static_cast<int>(round(144.f/2.f - x_*97.f + 1.f))); }
int DataIOStanford::to_y(float y_) { return std::min(176, static_cast<int>(round(176.f/2.f - y_*80.f + 5.f))); }

}
