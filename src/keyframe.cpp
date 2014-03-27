
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

	if (centre.x < window_size/2 || centre.x > keyframe_.cols - window_size/2
			|| centre.y < window_size/2 || centre.y > keyframe_.rows - window_size/2) return false;

	// Rotating the ROI so that the direction points downwards
	float dy = orientation[1] - orientation[3];
	float dx = orientation[0] - orientation[2];
	float angle = -acos(dy/sqrt(dy*dy + dx*dx)) * 180.f/M_PI;

	if (dx < 0) { angle = abs(angle); }

	cv::RotatedRect rrect(centre, cv::Size(window_size, window_size), angle);

	// try
	cv::Mat roi = keyframe_;

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
