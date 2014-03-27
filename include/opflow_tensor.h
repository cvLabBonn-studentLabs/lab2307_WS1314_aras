/*
 * opflow_tensor.h
 *
 *  Created on: Mar 26, 2014
 *      Author: araslanov
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
