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

#include "data_io_backend.h"

namespace io {

DataIOBackend::DataIOBackend() {}
DataIOBackend::~DataIOBackend() {}

bool DataIOBackend::read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth) { return false; }

void DataIOBackend::get_ground_truth(cv::Point& centre_head,
										cv::Point& centre_left_hand,
										cv::Point& centre_right_hand,
										cv::Point& centre_left_foot,
										cv::Point& centre_right_foot) {}

bool DataIOBackend::load_float_image(std::string filename, cv::Mat& output) {

	IplImage* imInt   = cvCreateImage( cvSize(176,144) , IPL_DEPTH_32F , 1);
	IplImage* img = 0;
	img = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	bool res = (img!=0);

	if(res) {

		// data for input image
		uchar* indata;
		int stepIn;
		cvGetRawData( img, (uchar**)&indata, &stepIn);
		int si = sizeof(indata[0]);
		stepIn /= si;

		// data for output image
		int so = sizeof(float);
		float* outdata;
		int stepOut;
		cvGetRawData( imInt, (uchar**)&outdata, &stepOut);
		stepOut /= so;

		// copy float -> uchar
		for( int y = 0; y < img->height; y++, indata += stepIn, outdata += stepOut) {
			int m = 0;
			for( int k=0; k < so; k++)
				for( int l = k; l < imInt->width*so; l+=so ) {
					*((uchar*)(outdata)+l*si) = *((uchar*)(indata) + m*si);
					m++;
				}
		}

		cvReleaseImage(&img);

	} else {

		std::cout << "Could not find " << filename << std::endl;
	}

	output = cv::Mat(imInt);

	return res;
}

void DataIOBackend::extract_parts() {}

void DataIOBackend::image_to_string(cv::Mat image, std::stringstream& line) {
	assert(image.type() == CV_32F);

	int index = 1;
	for(int row = 0; row < image.rows; ++row) {
		float* p = image.ptr<float>(row);
	    for(int col = 0; col < image.cols; ++col) {
	    	std::stringstream data;
	    	data << image.at<float>(row, col);
	    	line << " " << index++ << ":" << data.str();
	    }
	}
}


void DataIOBackend::extract_positive_part(cv::Mat& frame,
											std::ofstream& ostream,
											float x1_, float y1_,
											float x2_, float y2_,
											classifier::BodyPart id) {
	extract_positive_part(frame, ostream, x1_, y1_, x2_, y2_, id, false);
}

void DataIOBackend::extract_positive_part(cv::Mat& frame,
											std::ofstream& ostream,
											float x1_, float y1_,
											float x2_, float y2_,
											classifier::BodyPart id,
											bool mirror) {
	cv::Vec4i orientation;
	cv::Point centre;
	cv::Mat part;

	int x1 = to_x(x1_);
	int y1 = to_y(y1_);
	int x2 = to_x(x2_);
	int y2 = to_y(y2_);

	centre.x = x2;
	centre.y = y2;

	orientation[0] = x1;
	orientation[1] = y1;
	orientation[2] = x2;
	orientation[3] = y2;

	keyframe::Keyframe kframe(frame);
	kframe.extract_part(part, centre, orientation, pose::kDescriptorSize);

	if (part.cols == 0 && part.rows == 0) {
		std::cout << centre.x << " " << centre.y << std::endl;
		return;
	}

	std::stringstream line;
	image_to_string(part, line);
	ostream << id << line.str() << "\n";

	if (mirror) {
		cv::Mat part_mirror;
		cv::flip(part, part_mirror, 1);

		std::stringstream line;
		image_to_string(part_mirror, line);
		ostream << id << line.str() << "\n";
	}
}

void DataIOBackend::extract_negative_part(cv::Mat& frame,
											std::ofstream& ostream,
											std::vector<int> positives) {
	// Generating negative examples
	int k = 0;
	cv::Point topleft;
	while (k < io::kNegativeToPositiveRatio) {
		topleft.x = rand() % (frame.cols - pose::kDescriptorSize);
		topleft.y = rand() % (frame.rows - pose::kDescriptorSize);

		// Checking for 'collisions'
		bool in_collision = false;
		for (std::vector<int>::iterator it = positives.begin();
					it != positives.end(); ) {
			int pos_x = *it++ - pose::kDescriptorSize/2;
			int pos_y = *it++ - pose::kDescriptorSize/2;

			if (DataIOBackend::intersection_ratio(topleft, cv::Point(pos_x, pos_y)) > 0.5f) {
				in_collision = true;
				break;
			}
		}

		if (in_collision) continue;

		cv::Mat neg(frame, cv::Rect(topleft, cv::Size(pose::kDescriptorSize, pose::kDescriptorSize)));

		std::stringstream line;
		image_to_string(neg, line);
		ostream << -1 << line.str() << "\n";
		k++;
	}
}

}
