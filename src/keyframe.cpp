
#include "keyframe.h"

namespace keyframe {

Keyframe::Keyframe(cv::Mat frame) : keyframe_(frame) {}
Keyframe::~Keyframe() {}
/*
 * orientation := (x1, y1, x2, y2)^T
 *
 * */
bool Keyframe::extract_part(cv::Mat& dst,
								cv::Point centre, cv::Vec4i orientation,
								int window_size) {

	int align_x = 0;
	int align_y = 0;
	cv::Mat frame = keyframe_;

	if (centre.x < window_size/2 || centre.x > keyframe_.cols - window_size/2
			|| centre.y < window_size/2 || centre.y > keyframe_.rows - window_size/2) {

		align_x = window_size;
		align_y = window_size;

		frame = cv::Mat(cv::Size(frame.cols + 2*align_x, frame.rows + 2*align_y), CV_8UC1, cv::Scalar(128));
		cv::Mat roi = frame(cv::Rect(align_x, align_y, keyframe_.cols, keyframe_.rows));
		keyframe_.copyTo(roi);

		std::cout << "Aligned" << std::endl;
	}

	// Rotating the ROI so that the direction points downwards
	float dy = orientation[1] - orientation[3];
	float dx = orientation[0] - orientation[2];
	float angle = -acos(dy/sqrt(dy*dy + dx*dx)) * 180.f/M_PI;

	if (dx < 0) { angle = abs(angle); }

	centre.x += align_x;
	centre.y += align_y;
	cv::RotatedRect rrect(centre, cv::Size(window_size, window_size), angle);

	// try
	cv::Mat roi = frame;

	// get the rotation matrix
    cv::Mat R = cv::getRotationMatrix2D(rrect.center, angle, 1.0);

    // perform the affine transformation
    cv::Mat roi_rotated;
    warpAffine(roi, roi_rotated, R, roi.size(), cv::INTER_CUBIC);

    // crop the resulting image
    cv::getRectSubPix(roi_rotated, rrect.size, rrect.center, dst);

	return true;
};

}
