/*
 * backend.h
 *
 *  Created on: Mar 18, 2014
 *      Author: neek
 */

#ifndef BACKEND_STANFORD_H_
#define BACKEND_STANFORD_H_

#include "classifier_svm.h"
#include "data_io_backend.h"

namespace io {

class DataIOStanford : public DataIOBackend {
public:
	DataIOStanford();
	bool read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth);
	/*bool extract_part(const cv::Mat& src, cv::Mat& dst,
								cv::Point centre, cv::Vec4i orientation, int window_size);*/
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
};

}

#endif /* BACKEND_H_ */
