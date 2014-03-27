/*
 * opflow.h
 *
 *  Created on: Mar 26, 2014
 *      Author: araslanov
 */

#ifndef OPFLOW_H_
#define OPFLOW_H_

#include "CTensor.h"
#include "CMatrix.h"
#include "CFilter.h"

namespace opflow {

void merge_flows(const cv::Mat& forward_flow, const cv::Mat& backward_flow, cv::Mat& flow);
void computeCorners(CTensor<float>& aImage, CMatrix<float>& aCorners, float aRho);
void buildColorCode(CMatrix<float>& color_code);

class OpticalFlow {
	OpticalFlow();
	~OpticalFlow();
};

}

class CTrack {
public:
  CTrack() {mStopped = false; mLabel = -1;}
  std::vector<float> mx,my;             // current position of the track
  int mox,moy;                          // original starting point of the track
  int mLabel;                           // assignment to a region (ignored for tracking but makes the tracking files compatible to other tools I have)
  bool mStopped;                        // tracking stopped due to occlusion, etc.
  int mSetupTime;                       // Time when track was created
};


#endif /* OPFLOW_H_ */
