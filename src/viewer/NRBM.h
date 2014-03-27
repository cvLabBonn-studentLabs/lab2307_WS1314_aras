// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

// A collection of functions for RBM representations
// TWIST, MATRIX, AXIS-ANGLE 


#ifndef NRBMH
#define NRBMH

#include <CVector.h>
#include <CMatrix.h>
#include <vector>

namespace NRBM {
  // Transforms a rigid body motion in matrix representation to a twist representation
  void RBM2Twist(CVector<float> &T, CMatrix<float>& RBM);
  double RM2Twist(CMatrix<float>* RM,CVector<float>& T);
  void Twist2RBM(CVector<float>& aTwist, CMatrix<float>& A);
  void AdjointTwist(CVector<float> &T, CMatrix<float>& RBM);
  double Pitch(CVector<float> &TV);
  
  // Converts rigid body motion into rotation vector and translation vector
  void RBM2RVT(const CMatrix<float>* RBM, CVector<double>& rvt);
  void RVT2RBM(const CVector<double>* rvt, CMatrix<float>& RBM);
  // Converts rotation matrix into rotation vector representation
  void RM2RV(const CMatrix<float>* R, CVector<double>& rv);
  void RV2RM(const CVector<double>* rv, CMatrix<float>& R);
  // mean of weighted rotations
  void meanRotation(const std::vector<CMatrix<float> >* vR, const std::vector<double>* vW, CMatrix<float>& meanR);
  void invRBM(CMatrix<float>& RBM);
  void invRM(CMatrix<float>& R);
}

#endif
