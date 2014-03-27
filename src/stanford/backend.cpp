/*
 * backend.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: neek
 */

#include <boost/filesystem.hpp>
#include "keyframe.h"
#include "classifier_svm.h"
#include "stanford/backend.h"
#include "constants.h"
#include "read_data_eval.cpp"

namespace io {

const std::string DataIOStanford::kTrainingDataPath = "data/stanford/training/";

DataIOStanford::DataIOStanford() {}


bool DataIOStanford::read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth) {
//	while (true) {
		if(!stanford_eval::read_frame()) {  if (!stanford_eval::next_file()) return false; };
		stanford_eval::Frame f = stanford_eval::frame;

		cv::Mat frame(cv::Size(f.C, f.R), CV_32F);

		for (int i = 0; i < f.N; ++i) {
			frame.at<float>(i % f.R, i/f.R) = -(f.points[i][2] + 2.0);
		}

		cv::normalize(frame, frame, 0, 1, cv::NORM_MINMAX);
		frame.convertTo(frame_depth, CV_32F);

		for (int i = 0; i < f.N; ++i) {
			if (frame_depth.at<float>(i % f.R, i/f.R) > 0.7f)
				frame_depth.at<float>(i % f.R, i/f.R) = 0.3f;
		}

		// Ground truth
		if (f.markers[1].cond > 0) {
			centre_truth_head.x = to_x(f.markers[1].x);
			centre_truth_head.y = to_y(f.markers[1].y);
		} else {
			centre_truth_head.x = -1;
		}

		if (f.markers[24].cond > 0) {
			centre_truth_left_hand.x = to_x(f.markers[24].x);
			centre_truth_left_hand.y = to_y(f.markers[24].y);
		} else {
			centre_truth_left_hand.x = -1;
		}

		if (f.markers[16].cond > 0) {
			centre_truth_right_hand.x = to_x(f.markers[16].x);
			centre_truth_right_hand.y = to_y(f.markers[16].y);
		} else {
			centre_truth_right_hand.x = -1;
		}
		/*for (int j = 0; j < 30; j++) {
			std::cerr << j << " " << f.markers[j].x << ", "
					<< f.markers[j].y << ", " << f.markers[j].z << std::endl;
		}*/

		//float d = f.markers[1].z * (-0.200002);
		//float z = (f.markers[1].z * (-1.00002) - 1.) / d;
		//std::cerr << "cond = " << f.markers[25].cond << std::endl;
		//float x = f.markers[25].x;
		//float y = f.markers[25].y;

		// weird parameters (spent 2 f** hours finding 'em!)
		//float xx = 176/2 - x*90 - 15;
		//float yy = 144/2 - y*105 + 30;


//		cv::Vec4i orientation;
		// Left hand
		/*orientation[0] = to_x(f.markers[28].x);
		orientation[1] = to_y(f.markers[28].y);
		orientation[2] = to_x(f.markers[24].x);
		orientation[3] = to_y(f.markers[24].y);*/

		// Right hand

//		orientation[0] = to_x(f.markers[19].x);
//		orientation[1] = to_y(f.markers[19].y);
//		orientation[2] = to_x(f.markers[16].x);
//		orientation[3] = to_y(f.markers[16].y);
//
//		cv::Point centre(orientation[0], orientation[1]);
//
//		cv::Mat head;
//		extract_part(frame, head, centre, orientation, 41);

		/*std::cerr << "Marker(O): " << x
								<< " " << y << std::endl;
		std::cerr << "Marker(X): " << xx
				<< " " << yy << std::endl;

		cv::circle(frame,
				 cv::Point(orientation[0], orientation[1]),
				 1.0,
				 255.0,
				 -1,
				 8);

		cv::circle(frame,
				 cv::Point(orientation[2], orientation[3]),
				 1.0,
				 255.0,
				 -1,
				 8);

		cv::imshow("Stanford Eval", frame);
		cv::imshow("Stanford Eval: head", head);
		cv::waitKey(0);*/
//	}

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
	const std::string kTrainingDataPathHead = kTrainingDataPath + "head/";
	const std::string kTrainingDataPathHand = kTrainingDataPath + "hand/";
	const std::string kTrainingDataPathLeg = kTrainingDataPath + "leg/";

	boost::filesystem::path training_data_path_head(kTrainingDataPathHead);
	boost::filesystem::create_directory(training_data_path_head);

	boost::filesystem::path training_data_path_hand(kTrainingDataPathHand);
	boost::filesystem::create_directory(training_data_path_hand);

	boost::filesystem::path training_data_path_leg(kTrainingDataPathLeg);
	boost::filesystem::create_directory(training_data_path_leg);

	std::ofstream head_data((kTrainingDataPathHead + "data.txt").c_str());
	std::ofstream hand_data_left((kTrainingDataPathHand + "data_left.txt").c_str());
	std::ofstream hand_data_right((kTrainingDataPathHand + "data_right.txt").c_str());
	std::ofstream leg_data_left((kTrainingDataPathLeg + "data_left.txt").c_str());
	std::ofstream leg_data_right((kTrainingDataPathLeg + "data_right.txt").c_str());


	if (!head_data.is_open()) {
		std::cerr << "Unable to create training data files!" << std::endl;
	}

	int head_idx, hand_idx, leg_idx;
	head_idx = hand_idx = leg_idx = 1;
	while (true) {
		if(!stanford_eval::read_frame()) {  stanford_eval::next_file(); };
		stanford_eval::Frame f = stanford_eval::frame;

		cv::Mat frame(cv::Size(f.C, f.R), CV_32F);

		for (int i = 0; i < f.N; ++i) {
			frame.at<float>(i % f.R, i/f.R) = -(f.points[i][2] + 2.0);
		}

		cv::normalize(frame, frame, 0, 255, cv::NORM_MINMAX);
		frame.convertTo(frame, CV_8UC1);

		std::vector<int> positive_centres;

		// Head
		if (f.markers[3].cond > 0 && f.markers[1].cond > 0) {
			extract_positive_part(frame, head_data,
									f.markers[3].x, f.markers[3].y,
									f.markers[1].x, f.markers[1].y,
									classifier::HEAD);

			positive_centres.push_back(f.markers[3].x);
			positive_centres.push_back(f.markers[3].y);
		}

		// Left hand
		if (f.markers[28].cond > 0 && f.markers[24].cond > 0) {
			extract_positive_part(frame, head_data,
									f.markers[28].x, f.markers[28].y,
									f.markers[24].x, f.markers[24].y,
									classifier::LEFT_HAND);

			positive_centres.push_back(f.markers[28].x);
			positive_centres.push_back(f.markers[28].y);
		}

		// Right hand
		if (f.markers[19].cond > 0 && f.markers[16].cond > 0) {
			extract_positive_part(frame, head_data,
									f.markers[19].x, f.markers[19].y,
									f.markers[16].x, f.markers[16].y,
									classifier::RIGHT_HAND);

			positive_centres.push_back(f.markers[19].x);
			positive_centres.push_back(f.markers[19].y);
		}

		// Left leg
		/*if (f.markers[35].cond > 0 && f.markers[37].cond > 0) {
			extract_positive_part(frame, head_data,
									f.markers[35].x, f.markers[35].y,
									f.markers[37].x, f.markers[37].y,
									classifier::LEFT_FOOT);

			positive_centres.push_back(f.markers[35].x);
			positive_centres.push_back(f.markers[35].y);
		}

		// Right leg
		if (f.markers[43].cond > 0 && f.markers[41].cond > 0) {
			extract_positive_part(frame, head_data,
									f.markers[43].x, f.markers[43].y,
									f.markers[41].x, f.markers[41].y,
									classifier::RIGHT_FOOT);

			positive_centres.push_back(f.markers[43].x);
			positive_centres.push_back(f.markers[43].y);
		}*/

		extract_negative_part(frame, head_data, positive_centres);
	}
}

int DataIOStanford::to_x(float x_) { return static_cast<int>(round(176.f/2.f - x_*90.f - 15.f)); }
int DataIOStanford::to_y(float y_) { return static_cast<int>(round(144.f/2.f - y_*105.f + 30.f)); }

/*bool DataIOStanford::extract_part(const cv::Mat& src, cv::Mat& dst,
								cv::Point centre, cv::Vec4i orientation,
								int window_size) { return true; };*/

}
