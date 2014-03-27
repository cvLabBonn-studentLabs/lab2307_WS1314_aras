// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

#include <NRBM.h>
#include <NMath.h>

namespace NRBM {

  void RBM2Twist(CVector<float> &T, CMatrix<float>& RBM) {
    T.setSize(6);

    double theta = RM2Twist(&RBM,T);
    double invTheta = 1.0/theta;
    CVector<double> omega(3);
    omega(0) = T(3)*invTheta;
    omega(1) = T(4)*invTheta;
    omega(2) = T(5)*invTheta;
    
    CMatrix<double> omegaHat(3,3);
    omegaHat.data()[0] = 0.0;            omegaHat.data()[1] = -omega(2); omegaHat.data()[2] = omega(1);
    omegaHat.data()[3] = omega(2);  omegaHat.data()[4] = 0.0;            omegaHat.data()[5] = -omega(0);
    omegaHat.data()[6] = -omega(1); omegaHat.data()[7] = omega(0);  omegaHat.data()[8] = 0.0;
    
    CMatrix<double> omegaT(3,3);
    for (int j = 0; j < 3; j++)
      for (int i = 0; i < 3; i++)
	omegaT(i,j) = omega(i)*omega(j);
    
    //R = (omegaHat*(double)sin(theta))+((omegaHat*omegaHat)*(double)(1.0-cos(theta)));
    //R(0,0) += 1.0; R(1,1) += 1.0; R(2,2) += 1.0;
    CMatrix<double> R(3,3);
    for (int y = 0; y <= 2; y++)
      for (int x = 0; x <= 2; x++)
	R(x,y) = RBM(x,y);
      
    CMatrix<double> A(3,3);
    A.fill(0.0);
    A(0,0)=1.0; A(1,1)=1.0; A(2,2)=1.0;
    A -= R;  A*=omegaHat;  A+=omegaT*theta;

    CVector<double> p(3);
    p(0)=RBM(3,0);
    p(1)=RBM(3,1);
    p(2)=RBM(3,2);
    A.inv();
    CVector<double> v = A*p;      
    T(0)=float(v(0)*theta);
    T(1)=float(v(1)*theta);
    T(2)=float(v(2)*theta);
 
  }

  double RM2Twist(CMatrix<float>* RM,CVector<float>& T) {
    CVector<double> rv(3);
    RM2RV(RM,rv);
    T(3) = rv(0);
    T(4) = rv(1);
    T(5) = rv(2);
    return rv.norm();
  }

  // Twist2RBM
  void Twist2RBM(CVector<float>& aTwist, CMatrix<float>& A) {
    // Determine main body motion
    CVector<float> moment(3);
    moment(0) = aTwist(0); moment(1) = aTwist(1); moment(2) = aTwist(2);
    CVector<float> omega(3);
    omega(0) = aTwist(3); omega(1) = aTwist(4); omega(2) = aTwist(5);
    A.setSize(4,4);
    if ((omega(0)==0)&&(omega(1)==0)&&(omega(2)==0)) {
      A.data()[0] = 1.0; A.data()[1] = 0.0; A.data()[2] = 0.0; A.data()[3] = moment(0);
      A.data()[4] = 0.0; A.data()[5] = 1.0; A.data()[6] = 0.0; A.data()[7] = moment(1);
      A.data()[8] = 0.0; A.data()[9] = 0.0; A.data()[10]= 1.0; A.data()[11]= moment(2);
      A.data()[12]= 0.0; A.data()[13]= 0.0; A.data()[14]= 0.0; A.data()[15]= 1.0;
    }
    else {
      float theta = sqrt(omega(0)*omega(0)+omega(1)*omega(1)+omega(2)*omega(2));
      float invTheta = 0.0;
      if (theta != 0) invTheta = 1.0/theta;
      omega *= invTheta; moment *= invTheta;
      CMatrix<float> omegaHat(3,3);
      omegaHat.data()[0] = 0.0;       omegaHat.data()[1] = -omega(2); omegaHat.data()[2] = omega(1);
      omegaHat.data()[3] = omega(2);  omegaHat.data()[4] = 0.0;       omegaHat.data()[5] = -omega(0);
      omegaHat.data()[6] = -omega(1); omegaHat.data()[7] = omega(0);  omegaHat.data()[8] = 0.0;
      CMatrix<float> omegaT(3,3);
      for (int j = 0; j < 3; j++)
	for (int i = 0; i < 3; i++)
	  omegaT(i,j) = omega(i)*omega(j);
      CMatrix<float> R(3,3);
      R = (omegaHat*(float)sin(theta))+((omegaHat*omegaHat)*(float)(1.0-cos(theta)));
      R(0,0) += 1.0; R(1,1) += 1.0; R(2,2) += 1.0;
      CMatrix<float> T(R); T *= -1.0;
      T(0,0) += 1.0; T(1,1) += 1.0; T(2,2) += 1.0;
      CVector<float> t(3);
      t = T*(omega/moment)+((omegaT*moment)*theta);
      A.data()[0] = R.data()[0]; A.data()[1] = R.data()[1]; A.data()[2] = R.data()[2]; A.data()[3] = t(0);
      A.data()[4] = R.data()[3]; A.data()[5] = R.data()[4]; A.data()[6] = R.data()[5]; A.data()[7] = t(1);
      A.data()[8] = R.data()[6]; A.data()[9] = R.data()[7]; A.data()[10]= R.data()[8]; A.data()[11]= t(2);
      A.data()[12]= 0.0;         A.data()[13]= 0.0;         A.data()[14]= 0.0;         A.data()[15]= 1.0;
    }
  }

  // Tranbsform Twist with Rigid body motion ...
  void AdjointTwist(CVector<float> &T, CMatrix<float>& RBM) {
    CMatrix<float> XiHat(4,4);
    CMatrix<float> XiNew(4,4);
    
    CMatrix<float> RInv;
    RInv=RBM;
    RInv.inv();
    
    for(int i=0;i<4;i++)
      for(int j=0;j<4;j++)
	XiHat(i,j)=0.0;
    
    XiHat(3,0)=T(0);
    XiHat(3,1)=T(1);
    XiHat(3,2)=T(2);
    XiHat(0,1)=T(5);XiHat(1,0)=-T(5);
    XiHat(0,2)=-T(4);XiHat(2,0)=T(4);
    XiHat(1,2)=T(3);XiHat(2,1)=-T(3);
    
    XiNew=RBM*XiHat*RInv;

    T(0)=XiNew(3,0);
    T(1)=XiNew(3,1);
    T(2)=XiNew(3,2);
    T(3)=XiNew(1,2);
    T(4)=XiNew(2,0);
    T(5)=XiNew(0,1);  
    
  }

  double Pitch(CVector<float> &TV)
  {
    double h;
    double n;
    
    h=TV(0)*TV(3)+TV(1)*TV(4)+TV(2)*TV(5);
    
    n=sqrt(TV(3)*TV(3)+TV(4)*TV(4)+TV(5)*TV(5));
    if(n!=0)
      return h/n;
    else
      return 0;
  }
  
  // Converts rigid body motion into rotation vector and translation vector
  void RBM2RVT(const CMatrix<float>* RBM, CVector<double>& rvt) {
    RM2RV(RBM, rvt);
    rvt(3) = (*RBM)(3,0);
    rvt(4) = (*RBM)(3,1);
    rvt(5) = (*RBM)(3,2);
  }
  
  void RVT2RBM(const CVector<double>* rvt, CMatrix<float>& RBM) {
    RV2RM(rvt, RBM);
    RBM.data()[3] = (float)(*rvt)(3);
    RBM.data()[7] = (float)(*rvt)(4);
    RBM.data()[11] = (float)(*rvt)(5);
    RBM.data()[12] = 0.0f;
    RBM.data()[13] = 0.0f;
    RBM.data()[14] = 0.0f;
    RBM.data()[15] = 1.0f;
  }

  // new!!!

  // Converts rotation matrix into rotation vector representation
  void RM2RV(const CMatrix<float>* R, CVector<double>& rv) { 

    double d10 = (*R)(0,1)-(*R)(1,0);
    double d20 = (*R)(2,0)-(*R)(0,2);
    double d21 = (*R)(1,2)-(*R)(2,1);
    if ((fabs(d10)< 0.01) && 
	(fabs(d20)< 0.01) && 
	(fabs(d21)< 0.01)) {
      // singularity found
      // first check for identity matrix which must have +1 for all terms
      //  in leading diagonaland zero in other terms
      double a10 = (*R)(1,0)+(*R)(0,1);
      double a20 = (*R)(2,0)+(*R)(0,2);
      double a21 = (*R)(2,1)+(*R)(1,2);
      if ((fabs(a10) < 0.1) && 
	  (fabs(a20) < 0.1) && 
	  (fabs(a21) < 0.1) && 
	  (fabs((*R)(0,0)+(*R)(1,1)+(*R)(2,2)-3) < 0.1)) {
	// this singularity is identity matrix so angle = 0
	rv(0)=0.0; rv(1)=0.0; rv(2)=0.0;
      } else {
	// otherwise this singularity is angle = 180
	double xx = ((*R)(0,0)+1.0)/2.0;
	double yy = ((*R)(1,1)+1.0)/2.0;
	double zz = ((*R)(2,2)+1.0)/2.0;
	double xy = a10/4.0;
	double xz = a20/4.0;
	double yz = a21/4.0;
	if ((xx > yy) && (xx > zz)) { // (0,0) is the largest diagonal term
	  if (xx < 0.01) {
	    rv(0) = 0;
	    rv(1) = 0.7071*NMath::Pi;
	    rv(2) = 0.7071*NMath::Pi;
	  } else {
	    rv(0) = sqrt(xx)*NMath::Pi;
	    rv(1) = xy/rv(0)*NMath::Pi;
	    rv(2) = xz/rv(0)*NMath::Pi;
	  }
	} else if (yy > zz) { // (1,1) is the largest diagonal term
	  if (yy < 0.01) {
	    rv(0) = 0.7071*NMath::Pi;
	    rv(1) = 0;
	    rv(2) = 0.7071*NMath::Pi;
	} else {
	    rv(1) = sqrt(yy)*NMath::Pi;
	    rv(0) = xy/rv(1)*NMath::Pi;
	    rv(2) = yz/rv(1)*NMath::Pi;
	  }	
	} else { // (2,2) is the largest diagonal term so base result on this
	  if (zz < 0.01) {
	    rv(0) = 0.7071*NMath::Pi;
	    rv(1) = 0.7071*NMath::Pi;
	    rv(2) = 0;
	  } else {
	    rv(2) = sqrt(zz)*NMath::Pi;
	    rv(0) = xz/rv(2)*NMath::Pi;
	    rv(1) = yz/rv(2)*NMath::Pi;
	  }
	}
      }
      
    } else { // end singularities
      // as we have reached here there are no singularities so we can handle normally
      double theta = acos(((double)(*R)(0,0)+(double)(*R)(1,1)+(double)(*R)(2,2)-1.0)*0.5);
      theta /= sqrt(d21*d21+d20*d20+d10*d10); 

      rv(0) = d21*theta;
      rv(1) = d20*theta;
      rv(2) = d10*theta;
    }
  }
  
  void RV2RM(const CVector<double>* rv, CMatrix<float>& R) {
    double theta = sqrt((*rv)(0)*(*rv)(0)+(*rv)(1)*(*rv)(1)+(*rv)(2)*(*rv)(2));
    if (theta<0.01) {    
      R(0,0) = 1.0; R(1,0) = 0.0; R(2,0) = 0.0;
      R(0,1) = 0.0; R(1,1) = 1.0; R(2,1) = 0.0;
      R(0,2) = 0.0; R(1,2) = 0.0; R(2,2) = 1.0;
    } else {
      double invTheta = 1.0/theta;
      CVector<double> omega = (*rv)*invTheta;
      
      CMatrix<double> omegaHat(3,3);
      omegaHat.data()[0] = 0.0;       omegaHat.data()[1] = -omega(2); omegaHat.data()[2] = omega(1);
      omegaHat.data()[3] = omega(2);  omegaHat.data()[4] = 0.0;       omegaHat.data()[5] = -omega(0);
      omegaHat.data()[6] = -omega(1); omegaHat.data()[7] = omega(0);  omegaHat.data()[8] = 0.0;
      CMatrix<double> omegaT(3,3);
      for (int j = 0; j < 3; j++)
	for (int i = 0; i < 3; i++)
	  omegaT(i,j) = omega(i)*omega(j);
      omegaHat = (omegaHat*(double)sin(theta))+((omegaHat*omegaHat)*(double)(1.0-cos(theta)));
      R(0,0) = omegaHat(0,0) + 1.0; R(1,0) = omegaHat(1,0); R(2,0) = omegaHat(2,0);
      R(0,1) = omegaHat(0,1); R(1,1) = omegaHat(1,1) + 1.0; R(2,1) = omegaHat(2,1);
      R(0,2) = omegaHat(0,2); R(1,2) = omegaHat(1,2); R(2,2) = omegaHat(2,2) + 1.0;
    }  
  }

  // mean of rotations
  void meanRotation(const std::vector<CMatrix<float> >* vR, const std::vector<double>* vW, CMatrix<float>& meanR) {
    CMatrix<float> tmpRM(3,3);
    CVector<double> tmpRV(3);
    CVector<double> wRV(3);
    for(int t=0; t<5; ++t) {
      wRV.fill(0);
      CMatrix<float> invMeanR(trans(meanR));
      for(int i = 0; i<vR->size(); ++i) {
	tmpRM = invMeanR * (*vR)[i];
	RM2RV(&tmpRM, tmpRV);
	wRV += (*vW)[i]*tmpRV;
      }
      RV2RM(&wRV, tmpRM);
      meanR = meanR * tmpRM;
    }
  }

  void invRBM(CMatrix<float>& RBM) {
    CMatrix<float> tmp(RBM); 
    
    //RBM.data()[ 0] = tmp.data()[ 0];
    RBM.data()[ 1] = tmp.data()[ 4];
    RBM.data()[ 2] = tmp.data()[ 8];
    RBM.data()[ 3] = - tmp.data()[0] * tmp.data()[3] - tmp.data()[4] * tmp.data()[7] - tmp.data()[8] * tmp.data()[11];
    
    RBM.data()[ 4] = tmp.data()[ 1];
    //RBM.data()[ 5] = tmp.data()[ 5];
    RBM.data()[ 6] = tmp.data()[ 9];
    RBM.data()[ 7] = - tmp.data()[1] * tmp.data()[3] - tmp.data()[5] * tmp.data()[7] - tmp.data()[9] * tmp.data()[11];
    
    RBM.data()[ 8] = tmp.data()[ 2];
    RBM.data()[ 9] = tmp.data()[ 6];
    //RBM.data()[10] = tmp.data()[10];
    RBM.data()[11] = - tmp.data()[2] * tmp.data()[3] - tmp.data()[6] * tmp.data()[7] - tmp.data()[10] * tmp.data()[11];
    
    RBM.data()[12] = 0.0f;
    RBM.data()[13] = 0.0f;
    RBM.data()[14] = 0.0f;
    RBM.data()[15] = 1.0f;
    
  }

  void invRM(CMatrix<float>& R) {
    float tmp; 
    
    tmp = R.data()[ 1];
    R.data()[ 1] = R.data()[ 3];
    R.data()[ 3] = tmp;

    tmp = R.data()[ 2];
    R.data()[ 2] = R.data()[ 6];
    R.data()[ 6] = tmp;

    tmp = R.data()[ 5];
    R.data()[ 5] = R.data()[ 7];
    R.data()[ 7] = tmp;

  }

}

