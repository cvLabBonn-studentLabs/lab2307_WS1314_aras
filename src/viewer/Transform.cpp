// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

#include "Transform.h"

#include <iostream>
#include <fstream>

// OpenCV header
#include <cv.h>

using namespace std;

Transform::Transform(int w1, int h1) {  
  this->w1 = w1;
  this->h1 = h1;
}

Transform::~Transform() {
  cvReleaseMat(&cameraMatrix);
  cvReleaseMat(&distCoeffs);
}


void Transform::loadCameraConfig(const char* filename) {
  ifstream aStream(filename);
  if(aStream.is_open()) {
    float tmp;

    aStream>>fx_; aStream>>fy_; aStream>>cx_; aStream>>cy_;
    
    cameraMatrix = cvCreateMat(3, 3, CV_32FC1);
    cvmSet(cameraMatrix, 0, 0, (float) fx_);
    cvmSet(cameraMatrix, 1, 1, (float) fy_);
    cvmSet(cameraMatrix, 0, 2, (float) cx_);
    cvmSet(cameraMatrix, 1, 2, (float) cy_);
    cvmSet(cameraMatrix, 2, 2, 1.0);

    distCoeffs = cvCreateMat(1, 5, CV_32FC1);
    aStream >> tmp; cvmSet(distCoeffs, 0, 0, tmp);
    aStream >> tmp; cvmSet(distCoeffs, 0, 1, tmp);
    aStream >> tmp; cvmSet(distCoeffs, 0, 2, tmp);
    aStream >> tmp; cvmSet(distCoeffs, 0, 3, tmp);
    aStream >> tmp; cvmSet(distCoeffs, 0, 4, tmp);

    aStream>>r11;
    aStream>>r12;
    aStream>>r13;
    aStream>>r21;
    aStream>>r22;
    aStream>>r23;
    aStream>>r31;
    aStream>>r32;
    aStream>>r33;

    aStream>>t1;
    aStream>>t2;
    aStream>>t3;
    
  } else {
    cerr << "Error reading camera calibration file: " << filename << "\n";
    exit(-1);
  }
}

void Transform::undistort(IplImage* img_in, IplImage* img_out) {
  cvUndistort2(img_in, img_out, cameraMatrix, distCoeffs);
}






