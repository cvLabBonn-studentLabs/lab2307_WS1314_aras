/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Computer Science Institute III, University of Bonn
 *  Author: Nikita Araslanov, 26.03.2014
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

#ifndef OPFLOW_TENSOR_H_
#define OPFLOW_TENSOR_H_

#include <opencv2/opencv.hpp>
#include "CTensor.h"

namespace opflow {

template <class T>
class OFTensor : public CTensor<T> {
public:
	void readFromMat(const cv::Mat& image);
	void writeToMat(cv::Mat& image);
};

template <class T>
void OFTensor<T>::readFromMat(const cv::Mat& image) {
	assert(image.type() == cv::DataType<T>::type);

	this->mXSize = image.cols;
	this->mYSize = image.rows;
	this->mZSize = 1;

	this->mData = new T[this->mXSize*this->mYSize*this->mZSize];

	memcpy(this->mData, (T*)image.data, (this->mXSize * this->mYSize) * sizeof(T));
}

template <class T>
void OFTensor<T>::writeToMat(cv::Mat& image) {
	assert(image.type() == cv::DataType<T>::type);
	memcpy((T*)image.data, this->mData, (this->mXSize * this->mYSize) * sizeof(T));
}

}

#endif /* OPFLOW_TENSOR_H_ */
