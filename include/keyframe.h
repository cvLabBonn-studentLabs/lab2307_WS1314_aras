/*
 * keyframe.h
 *
 *  Created on: Mar 22, 2014
 *      Author: neek
 */

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <opencv2/opencv.hpp>

namespace keyframe {

class Keyframe {
public:
	Keyframe(cv::Mat src);
	~Keyframe();
	bool extract_part(cv::Mat& dst,
						cv::Point centre,
						cv::Vec4i orientation,
						int window_size);
private:
	cv::Mat keyframe_;
};

}


#endif /* KEYFRAME_H_ */
