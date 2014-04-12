/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Computer Science Institute III, University of Bonn
 *  Author: Nikita Araslanov, 22.03.2014
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

		frame = cv::Mat(cv::Size(frame.cols + 2*align_x, frame.rows + 2*align_y), CV_32F, 0.05);
		cv::Mat roi = frame(cv::Rect(align_x, align_y, keyframe_.cols, keyframe_.rows));
		keyframe_.copyTo(roi);
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
