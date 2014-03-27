// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

#ifndef Transform_H
#define Transform_H 

// OpenCV header
#include <cxcore.h>

class Transform {
 public:
  Transform(int w1, int h1);
  ~Transform();

  void loadCameraConfig(const char* filename);
  void undistort(IplImage* img_in, IplImage* img_out);

 private:
  CvMat* cameraMatrix;
  CvMat* distCoeffs;

  float fx_, fy_, cx_, cy_;
  float r11, r12, r13, r21, r22, r23, r31, r32, r33;
  float t1, t2, t3;

  int w1, h1;

};

#endif
