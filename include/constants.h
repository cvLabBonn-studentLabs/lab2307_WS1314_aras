/*
 * constants.h
 *
 *  Created on: Mar 9, 2014
 *      Author: neek
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace opflow {
	const bool ON = true;
}

namespace classifier {
	const float kConfidenceThreshold = 0.85f;
}

namespace io {
	enum Dataset { ETHZ, CornellCAD120, StanfordEval, Bonn };

	const Dataset kCurrentDataset = StanfordEval;

	const int kInputWidth = 144;
	const int kInputHeight = 176;

	const std::string kTrainingDataPath = "";
	const int kNegativeToPositiveRatio = 2;
}

namespace pose {

	const float kOpticalFlowThreshold = 90;//0.025f;
	const float kDownsampleCellSize = 3.3f;
	const float kMeshDistanceThreshold = 3.0f;
	const float kOrientationDelta = 20;

	const int kDescriptorSize = 41;
	const int kNumInterestPoints = 40;

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
