// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

#include <NShow.h>
#include <NMath.h>
#include <NRBM.h>
#include "CTMesh.h"

#include "GL/glew.h"

using namespace std;

// C J O I N T -----------------------------------------------------------------

// constructor
CJoint::CJoint(CVector<float>& aDirection, CVector<float>& aPoint, int aParent) {
  mDirection = aDirection;
  mPoint = aPoint;
  mMoment = aPoint/aDirection;
  mParent = aParent;
}

// rigidMotion
void CJoint::rigidMotion(CMatrix<float>& M) {
  CMatrix<float> R(3,3);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R(i,j) = M(i,j);
  mDirection = R*mDirection;
  mPoint = R*mPoint;
  mPoint(0) += M(3,0); mPoint(1) += M(3,1); mPoint(2) += M(3,2);
  mMoment = mPoint/mDirection;
}

// angleToMatrix
void CJoint::angleToMatrix(float aAngle, CMatrix<float>& M) {
  CMatrix<float> omegaHat(3,3);
  omegaHat.data()[0] = 0.0;            omegaHat.data()[1] = -mDirection(2); omegaHat.data()[2] = mDirection(1);
  omegaHat.data()[3] = mDirection(2);  omegaHat.data()[4] = 0.0;            omegaHat.data()[5] = -mDirection(0);
  omegaHat.data()[6] = -mDirection(1); omegaHat.data()[7] = mDirection(0);  omegaHat.data()[8] = 0.0;
  CMatrix<float> omegaT(3,3);
  for (int j = 0; j < 3; j++)
    for (int i = 0; i < 3; i++)
      omegaT(i,j) = mDirection(i)*mDirection(j);
  CMatrix<float> R(3,3);
  R = (omegaHat*(float)sin(aAngle))+((omegaHat*omegaHat)*(float)(1.0-cos(aAngle)));
  R(0,0) += 1.0; R(1,1) += 1.0; R(2,2) += 1.0;
  CMatrix<float> T(R); T *= -1.0;
  T(0,0) += 1.0; T(1,1) += 1.0; T(2,2) += 1.0;
  CVector<float> t(3);
  t = T*(mDirection/mMoment)+((omegaT*mMoment)*aAngle);
  M.data()[0] = R.data()[0]; M.data()[1] = R.data()[1]; M.data()[2] = R.data()[2]; M.data()[3] = t(0);
  M.data()[4] = R.data()[3]; M.data()[5] = R.data()[4]; M.data()[6] = R.data()[5]; M.data()[7] = t(1);
  M.data()[8] = R.data()[6]; M.data()[9] = R.data()[7]; M.data()[10]= R.data()[8]; M.data()[11]= t(2);
  M.data()[12]= 0.0;         M.data()[13]= 0.0;         M.data()[14]= 0.0;         M.data()[15]= 1.0;
}

// operator =
CJoint& CJoint::operator=(CJoint& aCopyFrom) {
  mDirection = aCopyFrom.mDirection;
  mPoint = aCopyFrom.mPoint;
  mMoment = aCopyFrom.mMoment;
  mParent = aCopyFrom.mParent;
  return *this;
}


// C M E S H -------------------------------------------------------------------

// constructor
CMesh::CMesh(int aPatches, int aPoints) {
  mPoints.resize(aPoints);
  mNormals.resize(aPoints);
  mPatch.resize(aPatches);
  mNumPoints=aPoints;
  mNumPatch=aPatches;

  for(int i=0;i<aPoints;i++)
    mPoints[i].setSize(4);

  for(int i=0;i<aPoints;i++)
    mNormals[i].setSize(3);

  for(int i=0;i<aPatches;i++)
    mPatch[i].setSize(3);

}

// readModel
bool CMesh::readModel(const char* aFilename, bool smooth) {
  cout << "Read Model... ";

  ifstream aStream(aFilename);
  if(!aStream.is_open()) {
    cerr << "Could not open " << aFilename << endl;
    return false;
  }

  int aXSize, aYSize, size = 4;

  CJoint Joint;

  aStream >> mNumPoints; 
  aStream >> mNumPatch; 
  aStream >> mJointNumber;
  if(smooth) {
    aStream >> mNumSmooth;
    if(mNumSmooth == 1)
      smooth = false;
    else
      size = 3 + mNumSmooth * 2;
  } else
    mNumSmooth = 1;
  
  cout << mNumPoints << " " << mNumPatch << " " << mJointNumber << " " << mNumSmooth << endl;

  CVector<bool> BoundJoints(mJointNumber+1,false);

  mJoint.setSize(mJointNumber+1);
  mJointLimits.resize(mJointNumber);
  
  // Read mesh components
  mPoints.resize(mNumPoints); 
  mNormals.resize(mNumPoints);
  mPatch.resize(mNumPatch);

  for(int i=0;i<mNumPoints;i++) {
    mPoints[i].setSize(size);
    mNormals[i].setSize(3);
  } 

  for(int i=0;i<mNumPatch;i++) {
    mPatch[i].setSize(3);
  }

  for (int i = 0; i < mNumPoints; i++) {
    aStream >> mPoints[i][0]; 
    aStream >> mPoints[i][1];
    aStream >> mPoints[i][2]; 
    if(smooth) {
      for(int n = 0; n < mNumSmooth * 2; n++)
        aStream >> mPoints[i][3 + n];
    } else
      aStream >> mPoints[i][3]; 
    BoundJoints((int)mPoints[i][3]) = true;
  }
  
  for (int i = 0; i < mNumPatch; i++) {
    aStream >> mPatch[i][0];
    aStream >> mPatch[i][1];
    aStream >> mPatch[i][2];
  }
  
  // set bounds
  int count = 0;
  mJointMap.setSize(mJointNumber+1);
  mJointMap = -1;
  for(int j = 0; j<=mJointNumber; ++j)
    if(BoundJoints(j))
      mJointMap(j) = count++; 

  cout << "Bodyparts: " << count << endl;
  mBounds.setSize(count,9,3);
  mNeighbor.setSize(count, count); 
  mBodyPartsPatches.resize(count);
  mBodyPartsPoints.resize(count);
  mBodyPartsJoints.resize(count);
  CMatrix<float> minV(count,3,100000);
  CMatrix<float> maxV(count,3,-100000);

 
  mCenter.setSize(4);
  mCenter[0] = 0;
  mCenter[1] = 0;
  mCenter[2] = 0;
  mCenter[3] = 1.0f;
   
  for (int i = 0; i < mNumPoints; i++) {
    int index = mJointMap((int)mPoints[i][3]);
    
    if ( mPoints[i][0]<minV(index,0) ) minV(index,0) = mPoints[i][0];
    if ( mPoints[i][1]<minV(index,1) ) minV(index,1) = mPoints[i][1];
    if ( mPoints[i][2]<minV(index,2) ) minV(index,2) = mPoints[i][2];
    if ( mPoints[i][0]>maxV(index,0) ) maxV(index,0) = mPoints[i][0];
    if ( mPoints[i][1]>maxV(index,1) ) maxV(index,1) = mPoints[i][1];
    if ( mPoints[i][2]>maxV(index,2) ) maxV(index,2) = mPoints[i][2];
  
    mCenter[0]+=mPoints[i][0];
    mCenter[1]+=mPoints[i][1];
    mCenter[2]+=mPoints[i][2];
  }
  
  mCenter[0] /= (float)mNumPoints;
  mCenter[1] /= (float)mNumPoints;
  mCenter[2] /= (float)mNumPoints;

  for(int i=0; i<count; ++i) {
    mBounds(i,0,0) = mBounds(i,1,0) = mBounds(i,2,0) = mBounds(i,3,0) = minV(i,0);
    mBounds(i,4,0) = mBounds(i,5,0) = mBounds(i,6,0) = mBounds(i,7,0) = maxV(i,0);
    mBounds(i,0,1) = mBounds(i,1,1) = mBounds(i,4,1) = mBounds(i,5,1) = minV(i,1);
    mBounds(i,2,1) = mBounds(i,3,1) = mBounds(i,6,1) = mBounds(i,7,1) = maxV(i,1);
    mBounds(i,0,2) = mBounds(i,2,2) = mBounds(i,4,2) = mBounds(i,6,2) = minV(i,2);
    mBounds(i,1,2) = mBounds(i,3,2) = mBounds(i,5,2) = mBounds(i,7,2) = maxV(i,2);
  }

  for(int j=0; j<mJointMap.size();++j)
    if(mJointMap(j)>=0)
      mBounds(mJointMap(j),8,0) = j;

  // get bodypart indices for patches and vertices

  count  = 0;
  for (int i = 0; i < mNumPatch; i++) {
    int tmp = mJointMap( std::min(std::min(mPoints[mPatch[i][0]][3],mPoints[mPatch[i][1]][3]),mPoints[mPatch[i][2]][3]) );
    if(tmp!=count) {
      mBodyPartsPatches[count] = i;
      count = tmp;
    }
  }
  mBodyPartsPatches[count] = mNumPatch;

  count  = 0;
  for (int i = 0; i < mNumPoints; i++) {
    int tmp = mJointMap(mPoints[i][3]);
    if(tmp!=count) {
      mBodyPartsPoints[count] = i;
      mBodyPartsJoints[count+1] = mPoints[i][3];
      count = tmp;
    }
  }
  mBodyPartsPoints[count] = mNumPoints;

  cout << "BodyPart F/V/J: ";
  for(int i=0; i<mBodyPartsPoints.size(); ++i)
    cout << mBodyPartsPatches[i] << "/" << mBodyPartsPoints[i] << "/" << mBodyPartsJoints[i] << " ";
  cout << endl;
 
  // Read joints
  int dummy;
  CVector<float> aDirection(3);
  CVector<float> aPoint(3);
  for (int aJointID = 1; aJointID <= mJointNumber; aJointID++) {
    aStream >> dummy; // ID
    aStream >> aDirection(0) >> aDirection(1) >> aDirection(2);
    aStream >> aPoint(0) >> aPoint(1) >> aPoint(2);
    mJoint(aJointID).set(aDirection,aPoint);
    aStream >> mJoint(aJointID).mParent;
    mJointLimits[aJointID-1].setSize(2);
    aStream >> mJointLimits[aJointID-1](0); aStream >> mJointLimits[aJointID-1](1);
  }
  // Determine which joint motion is influenced by parent joints
  mInfluencedBy.setSize(mJointNumber+1,mJointNumber+1);
  mInfluencedBy = false;
  for (int j = 0; j <= mJointNumber; j++)
    for (int i = 0; i <= mJointNumber; i++) {
      if (i == j) mInfluencedBy(i,j) = true;
      if (isParentOf(j,i)) mInfluencedBy(i,j) = true;
    }

  mNeighbor = false;
  for (int i = 0; i < mNeighbor.xSize(); i++) {
    mNeighbor(i,i) = true;
    int jID = (int)mBounds(i,8,0);
    for (int j = jID-1; j >= 0; --j) {
      if(mJointMap(j)>=0 && mInfluencedBy(jID,j)) {
	mNeighbor(i,mJointMap(j)) = true;
	mNeighbor(mJointMap(j),i) = true;
	break;
      }
    }
  }

  // Check for boundary points
  aStream >> dummy;
  mBoundaryPoints.resize(dummy);
  for(int i=0;i<dummy;++i) {
    aStream >> mBoundaryPoints[i];
  }
  cout << "Boundary Vertices: ";
  for(int i=0; i<mBoundaryPoints.size(); ++i)
    cout << mBoundaryPoints[i] << " ";
  cout << endl;

  // Check for end points
  aStream >> dummy;
  mEndPoints.resize(dummy);
  for(int k=0;k<mEndPoints.size();++k) {
    aStream >> dummy;
    mEndPoints[k].resize(dummy);
    for(int i=0;i<dummy;++i) {
      aStream >> mEndPoints[k][i];
    }
  }
  cout << "End Vertices: ";
  for(int k=0;k<mEndPoints.size();++k) {
    for(int i=0;i<mEndPoints[k].size();++i) {
      cout << mEndPoints[k][i] << " ";
    }
    cout << endl;
  }
  cout << endl;

  mRBM.identity(4);
  mJointAngles.setSize(mJointNumber);
  mJointAngles = 0; 

  cout << mNeighbor << endl;

  aStream.close();
  cout << "ok" << endl;

  return true;
}

void CMesh::generateNormals() {

  //cout << "Generate Normals" << endl;

  CVector<float> a(3);
  CVector<float> b(3);
  CVector<float> c(3);
  CVector<float> fn(3);
  int vIndex[3];

  // reset normals
  for (int i = 0; i < mNumPoints; i++)
    mNormals[i].fill(0);

  for (int i = 0; i < mNumPatch; i++) {
    GetPatch(i, vIndex[0], vIndex[1], vIndex[2]);
    GetPoint(vIndex[0], a[0], a[1], a[2]);
    GetPoint(vIndex[1], b[0], b[1], b[2]);
    GetPoint(vIndex[2], c[0], c[1], c[2]);

    fn = (b-a)/(c-a);
    float invN = 1.0f/fn.norm();   
    fn *= invN ;

    mNormals[ vIndex[0] ] += fn;  
    mNormals[ vIndex[1] ] += fn;
    mNormals[ vIndex[2] ] += fn;
  }

  for (int i = 0; i < mNumPoints; i++) {
    float invN = 1.0f/mNormals[i].norm();
    mNormals[i] *= invN;
  }

}


void CMesh::centerModel() {

  cout << "RESET CENTER!\n";

  CVector<float> trans(4);
  trans(0) = -mCenter(0); trans(1) = -mCenter(1); trans(2) = -mCenter(2); trans(3) = 0;
  cout << trans;

  for (int i = 0; i < mNumPoints; i++) {
    mPoints[i](0) += trans(0);
    mPoints[i](1) += trans(1);
    mPoints[i](2) += trans(2);
  }
  
  cout << "Joints:" << endl;
  for (int i = 1; i <= mJointNumber; i++) {
    CVector<float> jPos(mJoint[i].getPoint());
    for(int j = 0; j < 3; ++j) 
      jPos(j) += trans(j);
    cout << i << " " << jPos;
    mJoint[i].setPoint(jPos);
  }
  cout << endl;

  for(int i = 0; i < mBounds.xSize(); ++i) 
    for(int j = 0; j < mBounds.ySize()-1; ++j) {
      mBounds(i,j,0) += trans(0);
      mBounds(i,j,1) += trans(1);
      mBounds(i,j,2) += trans(2);
    }
  mCenter += trans;
  cout << mCenter;
}

// writeModel
void CMesh::writeModel(const char* aFilename) {
  cout << "Write Model... ";
  std::ofstream aStream(aFilename);
  aStream << "OFF" << std::endl;
  aStream << mNumPoints << " " << mNumPatch << " " << 0 << std::endl;
  // Write vertices
  for (int i = 0; i < mNumPoints; i++) {
    aStream << mPoints[i][0] << " ";   
    aStream << mPoints[i][1] << " ";
    aStream << mPoints[i][2] << std::endl;
  }
  // Write patches
  for (int i = 0; i < mNumPatch; i++) {
    aStream << "3 " << mPatch[i][0] << " ";
    aStream << mPatch[i][1] << " ";
    aStream << mPatch[i][2] << std::endl;
  }
  cout << "ok" << endl;
}

// readOFF
bool CMesh::readOFF(const char* aFilename) {
  cout << "Read OFF... ";
  std::ifstream aStream(aFilename);
  if(aStream.is_open()) {

    char buffer[200];
    aStream.getline(buffer,200);
    cout << buffer << endl;
    aStream >> mNumPoints;
    aStream >> mNumPatch;
    aStream >> mNumSmooth;
    mPoints.resize(mNumPoints);
    mPatch.resize(mNumPatch);

    mCenter.setSize(4);
    mCenter[0] = 0;
    mCenter[1] = 0;
    mCenter[2] = 0;
    mCenter[3] = 1.0f;

    mBounds.setSize(1,9,3);
    CVector<float> minV(3,100000);
    CVector<float> maxV(3,-100000);

    // Read vertices
    for (int i = 0; i < mNumPoints; i++) {
	for(int j=0; j<3; ++j) {
	  aStream >> mPoints[i][j];
	  mCenter[j] += mPoints[i][j];
	  if ( mPoints[i][j]<minV(j) ) minV(j) = mPoints[i][j];
	  if ( mPoints[i][j]>maxV(j) ) maxV(j) = mPoints[i][j];
	}
    }

    mCenter[0] /= (float)mNumPoints;
    mCenter[1] /= (float)mNumPoints;
    mCenter[2] /= (float)mNumPoints;

    mBounds(0,0,0) = mBounds(0,1,0) = mBounds(0,2,0) = mBounds(0,3,0) = minV(0);
    mBounds(0,4,0) = mBounds(0,5,0) = mBounds(0,6,0) = mBounds(0,7,0) = maxV(0);
    mBounds(0,0,1) = mBounds(0,1,1) = mBounds(0,4,1) = mBounds(0,5,1) = minV(1);
    mBounds(0,2,1) = mBounds(0,3,1) = mBounds(0,6,1) = mBounds(0,7,1) = maxV(1);
    mBounds(0,0,2) = mBounds(0,2,2) = mBounds(0,4,2) = mBounds(0,6,2) = minV(2);
    mBounds(0,1,2) = mBounds(0,3,2) = mBounds(0,5,2) = mBounds(0,7,2) = maxV(2);

    // Read triangles
    for (int i = 0; i < mNumPatch; i++) {
      int dummy;
      aStream >> dummy; 
      for(int j=0; j<3; ++j) {
	aStream >> mPatch[i][j];
      }
    }

    mJointNumber = 0;
    cout << "ok" << endl;
    return true;
  } else return false;
}

void CMesh::writeSkel(const char* aFilename) {
  cout << "Write Model... ";
  std::ofstream aStream(aFilename);
  aStream << "Skeleton" << std::endl;
  aStream << mJointNumber << std::endl;

  // Write vertices
  for (int i = 1; i <= mJointNumber; i++) {
    aStream << i << " ";   
    aStream << mJoint[i].getDirection() << " ";
    aStream << mJoint[i].getPoint() << " ";
    aStream << mJoint[i].mParent << std::endl;
  }

  cout << "ok" << endl;
}
  
// rigidMotion
void CMesh::rigidMotion(CVector<CMatrix<float> >& M,CVector<float>& X, bool smooth, bool force) {
 
  CVector<float> a(4); a(3) = 1.0;
  CVector<float> b(4);
  // Apply motion to points

  if(!smooth || mNumSmooth == 1) {

    for (int i = 0; i < mNumPoints; i++) {
      a(0) = mPoints[i](0); a(1) =  mPoints[i](1); a(2) = mPoints[i](2);
      b = M( int(mPoints[i](3)) )*a;
      mPoints[i](0)=b(0);mPoints[i](1)=b(1);mPoints[i](2)=b(2);
    }

  } else {

    for(int i = 0; i < mNumPoints; i++) {
      a(0) = mPoints[i](0); a(1) =  mPoints[i](1); a(2) = mPoints[i](2);
      b = 0;
      for(int n = 0; n < mNumSmooth; n++)
        b += M( int(mPoints[i](3 + n * 2)) ) * a * mPoints[i](4 + n * 2);
      mPoints[i](0)=b(0);mPoints[i](1)=b(1);mPoints[i](2)=b(2);
    }
  }
  
  // Apply motion to joints
#if 1
  for (int i = mJointNumber; i > 0; i--)
    mJoint(i).rigidMotion(M(mJoint(i).mParent));
#else
  for (int i = mJointNumber; i > 0; i--) {
    int j = mJoint(i).mParent;
    while (j > 0 && mJoint(j).getPoint() == mJoint(i).getPoint())
      j = mJoint(j).mParent;
    mJoint(i).rigidMotion(M(j));
  }
#endif
  
  if(!smooth || force) {
    for(int j=0;j<mBounds.xSize();++j) {
      int jID = (int)mBounds(j,8,0);
      for(int i=0;i<8;++i) {
	a(0) = mBounds(j,i,0); a(1) = mBounds(j,i,1); a(2) = mBounds(j,i,2); 
	b = M(jID)*a;
	mBounds(j,i,0) = b(0); mBounds(j,i,1) = b(1); mBounds(j,i,2) = b(2);
      }
    }
    
    mCenter = M(0)*mCenter;
    
    mRBM = M(0)*mRBM;
    mJointAngles += X;
  }
}

void CMesh::makeSmooth(CMesh* initMesh) {
  if(mNumSmooth>1) {

    mPoints = initMesh->mPoints;
    mJoint = initMesh->mJoint;
    
    CVector<float> X(mJointAngles);
    for(int i=0;i<X.size();++i) {
      if(X(i)>NMath::Pi) X(i)-=2*NMath::Pi;
      if(X(i)<-NMath::Pi) X(i)+=2*NMath::Pi;
    }
    mJointAngles = X;
      

    CVector<CMatrix<float> >M(joints()+1);
    
    M(0) = mRBM;
    
    for (int i = 1; i <= joints(); i++) {
      M(i).setSize(4,4); M(i) = 0;
      M(i)(0,0) = 1.0; M(i)(1,1) = 1.0; M(i)(2,2) = 1.0; M(i)(3,3) = 1.0;
    }
    
    for (int i = joints(); i > 0; i--) {
      CMatrix<float> Mi(4,4);
      joint(i).angleToMatrix(X(i-1),Mi);
      for (int j = 1; j <= joints(); j++) {
	if (influencedBy(j,i)) M(j) = Mi*M(j);
      }
    }
    
    for (int i = 1; i <= joints(); i++) {
      M(i) = M(0)*M(i);
    }
    
    rigidMotion(M,X, true);
  }
}

// angleToMatrix
void CMesh::angleToMatrix(const CMatrix<float>& aRBM, CVector<float>& aJAngles, CVector<CMatrix<float> >& M) const {
  // Determine motion of parts behind joints
  for (int i = 1; i <= mJointNumber; i++) {
    M(i).setSize(4,4); M(i) = 0;
    M(i)(0,0) = 1.0; M(i)(1,1) = 1.0; M(i)(2,2) = 1.0; M(i)(3,3) = 1.0;
  }
  for (int i = mJointNumber; i > 0; i--) {
    CMatrix<float> Mi(4,4);
    mJoint(i).angleToMatrix(aJAngles(i-1),Mi); // i-1
    for (int j = 1; j <= mJointNumber; j++)
      if (mInfluencedBy(j,i)) M(j) = Mi*M(j);
  }
  for (int i = 1; i <= mJointNumber; i++)
    M(i) = aRBM*M(i);
  M(0) = aRBM;
}

void CMesh::invAngleToMatrix(const CMatrix<float>& aRBM, CVector<float>& aJAngles, CVector<CMatrix<float> >& M) const {
  // Determine motion of parts behind joints
  for (int i = 1; i <= mJointNumber; i++) {
    M(i).setSize(4,4); M(i) = 0;
    M(i)(0,0) = 1.0; M(i)(1,1) = 1.0; M(i)(2,2) = 1.0; M(i)(3,3) = 1.0;
  }
  for (int i = mJointNumber; i > 0; i--) {
    CMatrix<float> Mi(4,4);
    mJoint(i).angleToMatrix(aJAngles(i-1),Mi); // i-1
    for (int j = 1; j <= mJointNumber; j++)
      if (mInfluencedBy(j,i)) M(j) = M(j)*Mi;
  }
  for (int i = 1; i <= mJointNumber; i++)
    M(i) = M(i)*aRBM;
  M(0) = aRBM;
}

void CMesh::twistToMatrix(CVector<float>& aTwist, CVector<CMatrix<float> >& M) {
  NRBM::Twist2RBM(aTwist,M(0));
  CVector<float> jangles(aTwist.size()-6);
  for(int i=0;i<jangles.size();++i)
    jangles[i] = aTwist[i+6];
  angleToMatrix(M(0), jangles, M);
}

// isParentOf
bool CMesh::isParentOf(int aParentJointID, int aJointID) const {
  if (aJointID == 0) return false;
  if (mJoint(aJointID).mParent == aParentJointID) return true;
  return isParentOf(aParentJointID,mJoint(aJointID).mParent);
}

// operator=
void CMesh::operator=(const CMesh& aMesh) {

  mJointNumber = aMesh.mJointNumber;
  
  mNumPoints=aMesh.mNumPoints;
  mNumPatch=aMesh.mNumPatch;
  mNumSmooth=aMesh.mNumSmooth;
  
  mPoints = aMesh.mPoints;
  mNormals = aMesh.mNormals;
  mPatch =  aMesh.mPatch;
  mBoundaryPoints =  aMesh.mBoundaryPoints;
  mEndPoints =  aMesh.mEndPoints;

  mBodyPartsPatches =  aMesh.mBodyPartsPatches;
  mBodyPartsPoints =  aMesh.mBodyPartsPoints;
  mBodyPartsJoints =  aMesh.mBodyPartsJoints;

  mJoint = aMesh.mJoint;
  mJointLimits = aMesh.mJointLimits;
  
  mBounds=aMesh.mBounds;
  mCenter=aMesh.mCenter;
  mJointMap=aMesh.mJointMap;
  mNeighbor=aMesh.mNeighbor;
  mInfluencedBy = aMesh.mInfluencedBy;

  mRBM = aMesh.mRBM;
  mJointAngles = aMesh.mJointAngles;
  
}


void CMesh::setupRenderBuffers() {

  if (!GLEW_ARB_vertex_buffer_object) {
    cerr << "Vertex Buffer Objects not supported!" << endl;
    exit(-1);
  }
  
  vbo_handle = -1;
  index_handle = -1;
  
  glGenBuffers(1, &vbo_handle);
  glGenBuffers(1, &index_handle);

  updateVertexBuffer();
  
  GLushort* iBuffer = new GLushort[mNumPatch * 3];
  
  int index = -1;
  for (unsigned int i=0; i<mNumPatch; i++) {
    iBuffer[++index] = (unsigned int)mPatch[i][0];
    iBuffer[++index] = (unsigned int)mPatch[i][1];
    iBuffer[++index] = (unsigned int)mPatch[i][2];
  }

  cout << glGetError() << endl;
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_handle);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, mNumPatch * 3 * sizeof(GLushort), iBuffer, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  cout << glGetError() << endl;

  delete[] iBuffer;
}

void CMesh::updateVertexBuffer() {
  //cout << "updateVertexBuffer" << endl;

  GLfloat* vnBuffer = new GLfloat[mNumPoints * 6];
  
  int index = -1;
  for (unsigned int i=0; i<mNumPoints; i++) {
    vnBuffer[++index] = mPoints[i][0];
    vnBuffer[++index] = mPoints[i][1];
    vnBuffer[++index] = mPoints[i][2];
    vnBuffer[++index] = mNormals[i][0];
    vnBuffer[++index] = mNormals[i][1];
    vnBuffer[++index] = mNormals[i][2];
  }
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo_handle);
  glBufferData(GL_ARRAY_BUFFER, mNumPoints * sizeof(GLfloat) * 6, vnBuffer, GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  
  delete[] vnBuffer;
}

void CMesh::render(int bodypart) {

  if(bodypart<0)
    bodypart = mBodyPartsPatches.size()-1;

#if 1

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_handle);
  glVertexPointer(3, GL_FLOAT, sizeof(GLfloat)*6, NULL);
  glNormalPointer(GL_FLOAT, sizeof(GLfloat)*6, (void*)(sizeof(GLfloat)*3) );

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_handle);

  glDrawElements(GL_TRIANGLES, mBodyPartsPatches[bodypart]*3, GL_UNSIGNED_SHORT, 0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glDisableClientState(GL_NORMAL_ARRAY);
  glDisableClientState( GL_VERTEX_ARRAY );

#else

  float a[3],b[3],c[3];
  int vIndex[3];

  for(int j = 0; j < mNumPatch; ++j) {	
    GetPatch(j, vIndex[0], vIndex[1], vIndex[2]);
    GetPoint(vIndex[0], a[0], a[1], a[2]);
    GetPoint(vIndex[1], b[0], b[1], b[2]);
    GetPoint(vIndex[2], c[0], c[1], c[2]);
    
    cout << a[0] << " " << a[1] << " " << a[2] << endl;
    cout << b[0] << " " << b[1] << " " << b[2] << endl;
    cout << c[0] << " " << c[1] << " " << c[2] << endl;

    glBegin (GL_TRIANGLES);
    glVertex3f (a[0], a[1], a[2]);
    glVertex3f (b[0], b[1], b[2]);
    glVertex3f (c[0], c[1], c[2]);
    glEnd();
  }

#endif

}

// void CMesh::renderDepth() {

//   float a[3],b[3],c[3];
//   int vIndex[3];

//   for(int j = 0; j < mNumPatch; ++j) {	
//     GetPatch(j, vIndex[0], vIndex[1], vIndex[2]);
//     GetPoint(vIndex[0], a[0], a[1], a[2]);
//     GetPoint(vIndex[1], b[0], b[1], b[2]);
//     GetPoint(vIndex[2], c[0], c[1], c[2]);

//     float depth = (a[2]+b[2]+c[2])/12000.0f;
//     glColor3f(depth,depth,depth);

//     glBegin (GL_TRIANGLES);
//     glVertex3f (a[0], a[1], a[2]);
//     glVertex3f (b[0], b[1], b[2]);
//     glVertex3f (c[0], c[1], c[2]);
//     glEnd();
//   }

// }


