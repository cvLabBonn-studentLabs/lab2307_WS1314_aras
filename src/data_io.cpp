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

#include "data_io.h"


typedef union
{
  struct
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

namespace io {

	DataIO::DataIO(Dataset dtype) : dtype_(dtype) {

		switch(dtype_)
		{
		case StanfordEval:
		{
			backend_ = boost::make_shared<DataIOStanford>();
		}
			break;
		case Bonn:
		{
			backend_ = boost::make_shared<DataIOBonn>();
		}
			break;
		default:
			break;
		}

	}

	DataIO::~DataIO() {}

	float DataIO::getScaleZ() {
		return backend_->getScaleZ();
	}

	bool DataIO::read(cv::Mat& depth_img) {

		cv::Mat rgb_img_dummy;
		if (backend_->read_next_frame(rgb_img_dummy, depth_img))
			return true;

		return false;
	}

	void DataIO::extract_data() {
		backend_->extract_parts();
	}

	void DataIO::get_ground_truth(cv::Point& centre_head,
											cv::Point& centre_left_hand,
											cv::Point& centre_right_hand,
											cv::Point& centre_left_foot,
											cv::Point& centre_right_foot) {
		backend_->get_ground_truth(centre_head, centre_left_hand, centre_right_hand, centre_left_foot, centre_right_foot);
	}

	bool DataIO::load_float_image(std::string filename, cv::Mat& output) {

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
}
