/*
 * data_io_backend.h
 *
 *  Created on: Mar 18, 2014
 *      Author: neek
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
		float isc_x = std::max(std::min(topleft1.x + pose::kDescriptorSize,
										topleft2.x + pose::kDescriptorSize)
								- std::max(topleft1.x, topleft2.x), 0);

		float isc_y = std::max(std::min(std::min(topleft1.y + pose::kDescriptorSize, io::kInputWidth - 1),
								std::min(topleft2.y + pose::kDescriptorSize, io::kInputWidth - 1))
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
