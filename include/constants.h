/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Computer Science Institute III, University of Bonn
 *  Author: Nikita Araslanov, 09.03.2014
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

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace opflow {
	const bool ON = false;
}

namespace classifier {
	extern float kConfidenceThreshold;
}

namespace io {
	enum Dataset { /*ETHZ, CornellCAD120,*/ StanfordEval, Bonn };

	const Dataset kCurrentDataset = Bonn;

	const int kInputWidth = 144;
	const int kInputHeight = 176;

	const std::string kTrainingDataPath = "";
	const int kNegativeToPositiveRatio = 2;
}

namespace pose {

	const float kOpticalFlowThreshold = 90;//0.025f;
	const float kMeshDistanceThreshold = 3.0f;
	const float kOrientationDelta = 8;

	const int kDescriptorSize = 41;
	const int kNumInterestPoints = 35;

	const int kConnectedComponentMinSize = 100;

}

namespace cad120 {

	const int SLEEP_TIME = 0;

	const int JOINT_NUM = 11;
	const int JOINT_DATA_ORI_NUM = 9;
	const int JOINT_DATA_POS_NUM = 3;
	const int JOINT_DATA_NUM = (JOINT_DATA_ORI_NUM + JOINT_DATA_POS_NUM);
	const int JOINT_DATA_TYPE_NUM = 2; // two types : orientation and xyz position

	const int TORSO_JOINT_NUM = 2;
	const int HEAD_JOINT_NUM = 0;

	const int POS_JOINT_NUM = 4;
	const int POS_JOINT_DATA_NUM = 3;

	const int POS_LEFT_HAND_NUM = 0;
	const int POS_RIGHT_HAND_NUM = 1;
	const int POS_LEFT_FOOT_NUM = 2;
	const int POS_RIGHT_FOOT_NUM = 3;

	const int X_RES = 640;
	const int Y_RES = 480;
	const int RGBD_data = 4;

	// 30 fps
	const int frameStoreNum = 66;
	const int compareFrame[] = {0, -5, -9, -14, -20, -27, -35, -44, -54, -65};
	const int compareFrameNum = sizeof(compareFrame)/sizeof(compareFrame[0]);

}


#endif /* CONSTANTS_H_ */
