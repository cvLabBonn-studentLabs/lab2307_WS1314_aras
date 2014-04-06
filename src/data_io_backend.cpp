/*
 * data_io_backend.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: neek
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
	assert(image.type() == CV_8UC1);

	int index = 1;
	for(int row = 0; row < image.rows; ++row) {
	    uchar* p = image.ptr(row);
	    for(int col = 0; col < image.cols; ++col) {
	    	//std::cout << index << ":" << static_cast<int>(*p);

	    	//if (index == 82) std::cout << ". 140 Done! " << line.str() << std::endl;
	    	std::stringstream data;
	    	data << static_cast<int>(image.at<uchar>(row, col));
	    	line << " " << index++ << ":" << data.str();
	    	//std::cout << ". Done. " << std::endl;
	    }
	}
}


void DataIOBackend::extract_positive_part(cv::Mat& frame,
											std::ofstream& ostream,
											float x1_, float y1_,
											float x2_, float y2_,
											classifier::BodyPart id) {
	cv::Vec4i orientation;
	cv::Point centre;
	cv::Mat part;
	int x1 = to_x(x1_);
	int y1 = to_y(y1_);
	int x2 = to_x(x2_);
	int y2 = to_y(y2_);

	centre.x = x2;
	centre.y = y2;

	if (id == 1) {	// HEAD
		orientation[0] = x2;
		orientation[1] = y2;
		orientation[2] = x1;
		orientation[3] = y1;
	} else {
		orientation[0] = x1;
		orientation[1] = y1;
		orientation[2] = x2;
		orientation[3] = y2;
	}

	keyframe::Keyframe kframe(frame);
	kframe.extract_part(part, centre, orientation, pose::kDescriptorSize);

	if (part.cols == 0 && part.rows == 0) {
		std::cout << centre.x << " " << centre.y << std::endl;
		return;
	}

	std::cout << "Extracted " << centre.x << " " << centre.y << std::endl;

//	cv::imshow("Part", part);
//	cv::waitKey(0);

	std::stringstream line;
	image_to_string(part, line);
	ostream << id << line.str() << "\n";
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
			int pos_x = *it++;
			int pos_y = *it++;
			if ((std::abs(topleft.x - pos_x) < pose::kDescriptorSize)
					|| (std::abs(topleft.y - pos_y) < pose::kDescriptorSize)) {
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
