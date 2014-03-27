#ifndef CMesh_H
#define CMesh_H

#include <fstream>
#include <CMatrix.h>
#include <CTensor.h>
#include <iostream>


class CJoint {
public:
  // constructor
  inline CJoint() {mParent = 0;mDirection.setSize(0);mPoint.setSize(0);mMoment.setSize(0);  };
  ~CJoint(){};
  inline CJoint(CJoint& aCopyFrom) {*this = aCopyFrom;};
  CJoint(CVector<float>& aDirection, CVector<float>& aPoint, int aParent);
  // Performs a rigid motion M of the joint
  void rigidMotion(CMatrix<float>& M);
  // Constructs the motion matrix from the joint axis and the rotation angle
  void angleToMatrix(float aAngle, CMatrix<float>& M);
  // Access to joint's position and axis
  inline void set(CVector<float>& aDirection, CVector<float>& aPoint) {mDirection = aDirection; mPoint = aPoint; mMoment = aPoint/aDirection;};
  inline void setDirection(CVector<float>& aDirection) {mDirection = aDirection; mMoment = mPoint/aDirection;};
  inline CVector<float>& getDirection() {return mDirection;};
  inline void setPoint(CVector<float>& aPoint) {mPoint = aPoint; mMoment = aPoint/mDirection;};
  inline CVector<float>& getPoint() {return mPoint;};
  inline CVector<float>& getMoment() {return mMoment;};
  // Copy operator
  CJoint& operator=(CJoint& aCopyFrom);
  // Parent joint
  int mParent;
protected:
  // Defines joint's position and axis
  CVector<float> mDirection;
  CVector<float> mPoint;
  CVector<float> mMoment;
};

class CMesh {
public:
  // constructor
  CMesh() {};
  CMesh(const CMesh& aMesh);
  CMesh(int aPoints, int aPatches);
  // destructor
  ~CMesh() {};
  // Reads the mesh from a file
  bool readModel(const char* aFilename, bool smooth = false);
  bool readOFF(const char* aFilename);
  void centerModel();
  void generateNormals();
  // Writes the mesh to a file
  void writeModel(const char* aFilename);
  void writeSkel(const char* aFilename);
 
  // Performs rigid body motions M of the mesh points in the kinematic chain
  void rigidMotion(CVector<CMatrix<float> >& M,CVector<float>& X, bool smooth = false, bool force = false);
  // Reuses InitMesh to set up Smooth Pose: Global transformation
  void makeSmooth(CMesh* initMesh);
  
  // Functions have been changed !!!
  void angleToMatrix(const CMatrix<float>& aRBM, CVector<float>& aJAngles, CVector<CMatrix<float> >& M) const;
  void invAngleToMatrix(const CMatrix<float>& aRBM, CVector<float>& aJAngles, CVector<CMatrix<float> >& M) const;
  void twistToMatrix(CVector<float>& aTwist, CVector<CMatrix<float> >& M);

  // Fast projection of a 3-D point to the image plane
  void projectPoint(CMatrix<float>& P, float X, float Y, float Z, int& x, int& y) const;
  void projectPoint(CMatrix<float>& P, float X, float Y, float Z, float& x, float& y) const;

  // Copies aMesh
  void operator=(const CMesh& aMesh);

  // Returns the number of joints
  int joints() const {return mJointNumber;};
  void setJointDir(int aJointID, CVector<float> dir) {
    mJoint(aJointID).setDirection(dir);
  };
  // Returns a joint
  CJoint& joint(int aJointID) const {return mJoint(aJointID);};
  // Returns whether a point is influenced by a certain joint
  bool influencedBy(int aJointIDOfPoint, int aJointID) const { return mInfluencedBy(aJointIDOfPoint,aJointID);};
  bool isNeighbor(int i,int j) const {return mNeighbor(mJointMap(i),mJointMap(j));};
 
  void GetPoint(int i,  float& x, float& y, float& z) const;
  void GetPoint(int i,  float& x, float& y, float& z, int& j) const;
  void GetPoint(int i,  float& x, float& y, float& z, float& j) const;
  void GetNormal(int i,  float& x, float& y, float& z) const;
  void GetPatch(int i, int& x, int& y, int& z) const;
  void GetBounds(int J, int i, float& x, float& y, float& z) const;
  int GetBoundJID(int J) const {return (int)mBounds(J,8,0);};
  float GetCenter(int i) const {return mCenter[i];};
  int GetBodyPart(int jID) const {return mJointMap[jID];};

  int GetMeshSize() const {return mNumPatch;};
  int GetPointSize() const {return mNumPoints;};
  int GetPointSize(int bp) const {
    if(bp<0) bp = mBodyPartsPoints.size()-1;
    return mBodyPartsPoints[bp];
  };
  int GetJointSize(int bp) const {
    if(bp<0) bp = mBodyPartsJoints.size()-1;
    return mBodyPartsJoints[bp];
  };
  float GetJLowerBound(int i) {return mJointLimits[i](0);}
  float GetJUpperBound(int i) {return mJointLimits[i](1);}

  // body parts
  int GetBoundSize() const {return mBounds.xSize();};

  int GetJointID(int i) const {return (int)mPoints[i](3);};

  // rendering
  void setupRenderBuffers();
  void updateVertexBuffer();
  void render(int bodypart = -1);
  
  // Main body motion 
  CMatrix<float> mRBM; 
  // Vector with joint angles
  CVector<float> mJointAngles; 

  std::vector<int>  mBoundaryPoints;
  std::vector<std::vector<int> >  mEndPoints;

 protected:
  
  int mJointNumber;

  int mNumPoints;
  int mNumPatch;
  int mNumSmooth; 
  
  std::vector<CVector<float> >  mPoints;
  std::vector<CVector<float> >  mNormals;
  std::vector<CVector<int> >  mPatch;

  std::vector<int>  mBodyPartsPatches;
  std::vector<int>  mBodyPartsPoints;
  std::vector<int>  mBodyPartsJoints;

  CVector<CJoint>  mJoint;
  std::vector<CVector<float> > mJointLimits;

  CTensor<float> mBounds;
  CVector<float> mCenter;
  CVector<int> mJointMap;
  CMatrix<bool> mNeighbor;
  // how many joints can influence any given point
  CMatrix<bool> mInfluencedBy;
  
  // true if aParentJoint is an ancestor of aJoint
  bool isParentOf(int aParentJointID, int aJointID) const;

  // rendering
  unsigned int vbo_handle;
  unsigned int index_handle;

};

inline CMesh::CMesh(const CMesh& aMesh) {
  *this = aMesh;
}

// projectPoint
inline void CMesh::projectPoint(CMatrix<float>& P, float X, float Y, float Z, int& x, int& y) const {
  
  float hx = P.data()[0]*X + P.data()[1]*Y + P.data()[2]*Z + P.data()[3];
  float hy = P.data()[4]*X + P.data()[5]*Y + P.data()[6]*Z + P.data()[7];
  float hz = P.data()[8]*X + P.data()[9]*Y + P.data()[10]*Z + P.data()[11];
  
  
  float invhz = 1.0/hz;
  x = (int)(hx*invhz+0.5);
  y = (int)(hy*invhz+0.5);
}

inline void CMesh::projectPoint(CMatrix<float>& P, float X, float Y, float Z, float& x, float& y) const {
  
  float hx = P.data()[0]*X + P.data()[1]*Y + P.data()[2]*Z + P.data()[3];
  float hy = P.data()[4]*X + P.data()[5]*Y + P.data()[6]*Z + P.data()[7];
  float hz = P.data()[8]*X + P.data()[9]*Y + P.data()[10]*Z + P.data()[11];
  
  float invhz = 1.0/hz;
  x = hx*invhz;
  y = hy*invhz;
}

inline void CMesh::GetPoint(int i,  float& x, float& y, float& z) const {
  x=mPoints[i](0);
  y=mPoints[i](1);
  z=mPoints[i](2);
}

inline void CMesh::GetNormal(int i,  float& x, float& y, float& z) const {
  x=mNormals[i](0);
  y=mNormals[i](1);
  z=mNormals[i](2);
}

inline void CMesh::GetPatch(int i, int& x, int& y, int& z) const {
  x=mPatch[i](0);
  y=mPatch[i](1);
  z=mPatch[i](2);   
}


inline void CMesh::GetBounds(int J, int i,  float& x, float& y, float& z) const {
  x=mBounds(J,i,0);
  y=mBounds(J,i,1);
  z=mBounds(J,i,2);
}

inline void CMesh::GetPoint(int i,  float& x, float& y, float& z, int& j) const {
  x=mPoints[i](0);
  y=mPoints[i](1);
  z=mPoints[i](2);
  j=int(mPoints[i](3));
}

inline void CMesh::GetPoint(int i,  float& x, float& y, float& z, float& j) const {
  x=mPoints[i](0);
  y=mPoints[i](1);
  z=mPoints[i](2);
  j=mPoints[i](3);
}

#endif

