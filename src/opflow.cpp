/*
 * opflow.cpp
 *
 *  Created on: Mar 26, 2014
 *      Author: araslanov
 */

#include <opencv2/opencv.hpp>
#include "opflow.h"

namespace opflow {

void merge_flows(const cv::Mat& forward_flow, const cv::Mat& backward_flow, cv::Mat& flow) {
	// TODO: gpu::max?
	flow = cv::max(forward_flow, backward_flow);
}

void computeCorners(CTensor<float>& aImage, CMatrix<float>& aCorners, float aRho) {
  aCorners.setSize(aImage.xSize(),aImage.ySize());
  int aXSize = aImage.xSize();
  int aYSize = aImage.ySize();
  int aSize = aXSize*aYSize;
  // Compute gradient
  CTensor<float> dx(aXSize,aYSize,aImage.zSize());
  CTensor<float> dy(aXSize,aYSize,aImage.zSize());
  CDerivative<float> aDerivative(3);
  NFilter::filter(aImage,dx,aDerivative,1,1);
  NFilter::filter(aImage,dy,1,aDerivative,1);
  // Compute second moment matrix
  CMatrix<float> dxx(aXSize,aYSize,0);
  CMatrix<float> dyy(aXSize,aYSize,0);
  CMatrix<float> dxy(aXSize,aYSize,0);
  int i2 = 0;
  for (int k = 0; k < aImage.zSize(); k++)
    for (int i = 0; i < aSize; i++,i2++) {
      dxx.data()[i] += dx.data()[i2]*dx.data()[i2];
      dyy.data()[i] += dy.data()[i2]*dy.data()[i2];
      dxy.data()[i] += dx.data()[i2]*dy.data()[i2];
    }
  // Smooth second moment matrix
  NFilter::recursiveSmoothX(dxx,aRho);
  NFilter::recursiveSmoothY(dxx,aRho);
  NFilter::recursiveSmoothX(dyy,aRho);
  NFilter::recursiveSmoothY(dyy,aRho);
  NFilter::recursiveSmoothX(dxy,aRho);
  NFilter::recursiveSmoothY(dxy,aRho);
  // Compute smallest eigenvalue
  for (int i = 0; i < aSize; i++) {
    float a = dxx.data()[i];
    float b = dxy.data()[i];
    float c = dyy.data()[i];
    float temp = 0.5*(a+c);
    float temp2 = temp*temp+b*b-a*c;
    if (temp2 < 0.0f) aCorners.data()[i] = 0.0f;
    else aCorners.data()[i] = temp-sqrt(temp2);
  }
}


void buildColorCode(CMatrix<float>& colour_code) {
  colour_code.setSize(3,15000);
  for (int i = 0; i < 256; i++) {
    colour_code(0,i) = 0;
    colour_code(1,i) = i;
    colour_code(2,i) = 255;
  }
  for (int i = 0; i < 256; i++) {
    colour_code(0,i+256) = 0;
    colour_code(1,i+256) = 255;
    colour_code(2,i+256) = 255-i;
  }
  for (int i = 0; i < 256; i++) {
    colour_code(0,i+512) = i;
    colour_code(1,i+512) = 255;
    colour_code(2,i+512) = 0;
  }
  for (int i = 0; i < 256; i++) {
    colour_code(0,i+768) = 255;
    colour_code(1,i+768) = 255-i;
    colour_code(2,i+768) = 0;
  }
  for (int i = 0; i < 256; i++) {
    colour_code(0,i+1024) = 255;
    colour_code(1,i+1024) = 0;
    colour_code(2,i+1024) = i;
  }
  for (int i = 1280; i < colour_code.ySize(); i++) {
    colour_code(0,i) = 255;
    colour_code(1,i) = 0;
    colour_code(2,i) = 255;
  }
}

}
