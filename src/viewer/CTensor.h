// CTensor
// A three-dimensional array
//
// Author: Thomas Brox
//-------------------------------------------------------------------------
// Linker problems occured in connection with <vector> from the STL.
// In this case you should include this file in a namespace.
// Example:
// namespace NTensor {
//   #include <CTensor.h>
// }
// After including other packages you can then write:
// using namespace NTensor;

#ifndef CTENSOR_H
#define CTENSOR_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "CMatrix.h"
#include <png.h>

template <class T>
class CTensor {
public:
  // standard constructor
  inline CTensor();
  // constructor
  inline CTensor(const int aXSize, const int aYSize, const int aZSize);
  // copy constructor
  CTensor(const CTensor<T>& aCopyFrom);
  // constructor with implicit filling
  CTensor(const int aXSize, const int aYSize, const int aZSize, const T aFillValue);
  // destructor
  virtual ~CTensor();

  // Changes the size of the tensor, data will be lost
  void setSize(int aXSize, int aYSize, int aZSize);
  // Downsamples the tensor
  void downsample(int aNewXSize, int aNewYSize);
  // Upsamples the tensor
  void upsample(int aNewXSize, int aNewYSize);
  void upsampleBilinear(int aNewXSize, int aNewYSize);
  // Fills the tensor with the value aValue (see also operator =)
  void fill(const T aValue);
  // Fills a circle with the value aValue
  void fillCircle(const T aValue1, const T aValue2, const T aValue3, int x, int y, int r);
  // Copies a box from the tensor into aResult, the size of aResult will be adjusted
  void cut(CTensor<T>& aResult, int x1, int y1, int z1, int x2, int y2, int z2);
  void cut(CMatrix<T>& aResult, int x1, int y1, int x2, int y2, int z);
  // Copies aCopyFrom at a certain position of the tensor
  void paste(CTensor<T>& aCopyFrom, int ax, int ay, int az);
  // Mirrors the boundaries, aFrom is the distance from the boundaries where the pixels are copied from,
  // aTo is the distance from the boundaries they are copied to
  void mirrorLayers(int aFrom, int aTo);
  // Transforms the values so that they are all between aMin and aMax
  // aInitialMin/Max are initializations for seeking the minimum and maximum, change if your
  // data is not in this range or the data type T cannot hold these values
  void normalizeEach(T aMin, T aMax, T aInitialMin = -30000, T aInitialMax = 30000);
  void normalize(T aMin, T aMax, int aChannel, T aInitialMin = -30000, T aInitialMax = 30000);
  void normalize(T aMin, T aMax, T aInitialMin = -30000, T aInitialMax = 30000);
  // Draws a line into the image (only for mZSize = 3)
  void drawLine(int dStartX, int dStartY, int dEndX, int dEndY, T aValue1 = 255, T aValue2 = 255, T aValue3 = 255);
  // Reads the tensor from a file in Mathematica format
  void readFromMathematicaFile(const char* aFilename);
  // Writes the tensor to a file in Mathematica format
  void writeToMathematicaFile(const char* aFilename);
  // Reads the tensor from a movie file in IM format
  void readFromIMFile(const char* aFilename);
  // Writes the tensor to a movie file in IM format
  void writeToIMFile(const char* aFilename);
  // Reads an image from a PGM file
  void readFromPGM(const char* aFilename);
  // Writes the tensor in PGM-Format
  void writeToPGM(const char* aFilename);
  // Reads a color image from a PNG file
  void readFromPNG(const char* aFilename);
  // Writes the tensor in PPM-Format
  void writeToPNG(const char* aFilename);
  // Reads a color image from a PNG file
  void readFromPNGflip(const char* aFilename);
  // Writes the tensor in PPM-Format
  void writeToPNGflip(const char* aFilename);
  // Extends a XxYx1 tensor to a XxYx3 tensor with three identical layers
  void makeColorTensor();
  // Reads a color image from a PPM file
  void readFromPPM(const char* aFilename);
  // Writes the tensor in PPM-Format
  void writeToPPM(const char* aFilename);
  // Reads the tensor from a PDM file
  void readFromPDM(const char* aFilename);
  // Writes the tensor in PDM-Format
  void writeToPDM(const char* aFilename, char aFeatureType);

  // Gives full access to tensor's values
  inline T& operator()(const int ax, const int ay, const int az) const;
  // Read access with bilinear interpolation
  CVector<T> operator()(const float ax, const float ay) const;
  // Fills the tensor with the value aValue (equivalent to fill())
  inline CTensor<T>& operator=(const T aValue);
  // Copies the tensor aCopyFrom to this tensor (size of tensor might change)
  CTensor<T>& operator=(const CTensor<T>& aCopyFrom);
  // Adds a tensor of same size
  CTensor<T>& operator+=(const CTensor<T>& aMatrix);
  // Adds a constant to the tensor
  CTensor<T>& operator+=(const T aValue);
  // Multiplication with a scalar
  CTensor<T>& operator*=(const T aValue);

  // Returns the minimum value
  T min() const;
  // Returns the maximum value
  T max() const;
  // Returns the average value
  T avg() const;
  // Returns the average value of a specific layer
  T avg(int az) const;
  // Gives access to the tensor's size
  inline int xSize() const;
  inline int ySize() const;
  inline int zSize() const;
  inline int size() const;
  // Returns the az layer of the tensor as matrix (slow and fast version)
  CMatrix<T> getMatrix(const int az) const;
  void getMatrix(CMatrix<T>& aMatrix, const int az) const;
  // Copies the matrix components of aMatrix into the az layer of the tensor
  void putMatrix(CMatrix<T>& aMatrix, const int az);
  // Gives access to the internal data representation (use sparingly)
  inline T* data() const;

  // Possible interpretations of the third tensor dimension for PDM format
  static const char cSpacial = 'S';
  static const char cVector = 'V';
  static const char cColor = 'C';
  static const char cSymmetricMatrix = 'Y';
protected:
  int mXSize,mYSize,mZSize;
  T *mData;
};

// Provides basic output functionality (only appropriate for very small tensors)
template <class T> std::ostream& operator<<(std::ostream& aStream, const CTensor<T>& aTensor);

// Exceptions thrown by CTensor-------------------------------------------------

// Thrown when one tries to access an element of a tensor which is out of
// the tensor's bounds
struct ETensorRangeOverflow {
  ETensorRangeOverflow(const int ax, const int ay, const int az) {
    using namespace std;
    cerr << "Exception ETensorRangeOverflow: x = " << ax << ", y = " << ay << ", z = " << az << endl;
  }
};

// Thrown when the size of a tensor does not match the needed size for a certain operation
struct ETensorIncompatibleSize {
  ETensorIncompatibleSize(int ax, int ay, int ax2, int ay2) {
    using namespace std;
    cerr << "Exception ETensorIncompatibleSize: x = " << ax << ":" << ax2;
    cerr << ", y = " << ay << ":" << ay2 << endl;
  }
  ETensorIncompatibleSize(int ax, int ay, int az) {
    std::cerr << "Exception ETensorIncompatibleTensorSize: x = " << ax << ", y = " << ay << ", z= " << az << std::endl;
  }
};

// I M P L E M E N T A T I O N --------------------------------------------
//
// You might wonder why there is implementation code in a header file.
// The reason is that not all C++ compilers yet manage separate compilation
// of templates. Inline functions cannot be compiled separately anyway.
// So in this case the whole implementation code is added to the header
// file.
// Users of CTensor should ignore everything that's beyond this line :)
// ------------------------------------------------------------------------

// P U B L I C ------------------------------------------------------------

// standard constructor
template <class T>
inline CTensor<T>::CTensor() {
  mData = 0; mXSize = mYSize = mZSize = 0;
}

// constructor
template <class T>
inline CTensor<T>::CTensor(const int aXSize, const int aYSize, const int aZSize)
  : mXSize(aXSize), mYSize(aYSize), mZSize(aZSize) {
  mData = new T[aXSize*aYSize*aZSize];
}

// copy constructor
template <class T>
CTensor<T>::CTensor(const CTensor<T>& aCopyFrom)
  : mXSize(aCopyFrom.mXSize), mYSize(aCopyFrom.mYSize), mZSize(aCopyFrom.mZSize) {
  int wholeSize = mXSize*mYSize*mZSize;
  mData = new T[wholeSize];
  for (register int i = 0; i < wholeSize; i++)
    mData[i] = aCopyFrom.mData[i];
}

// constructor with implicit filling
template <class T>
CTensor<T>::CTensor(const int aXSize, const int aYSize, const int aZSize, const T aFillValue)
  : mXSize(aXSize), mYSize(aYSize), mZSize(aZSize) {
  mData = new T[aXSize*aYSize*aZSize];
  fill(aFillValue);
}

// destructor
template <class T>
CTensor<T>::~CTensor() {
  delete [] mData;
}

// setSize
template <class T>
void CTensor<T>::setSize(int aXSize, int aYSize, int aZSize) {
  if (mData != 0) delete[] mData;
  mData = new T[aXSize*aYSize*aZSize];
  mXSize = aXSize;
  mYSize = aYSize;
  mZSize = aZSize;
}

//downsample
template <class T>
void CTensor<T>::downsample(int aNewXSize, int aNewYSize) {
  T* mData2 = new T[aNewXSize*aNewYSize*mZSize];
  int aSize = aNewXSize*aNewYSize;
  for (int z = 0; z < mZSize; z++) {
    CMatrix<T> aTemp(mXSize,mYSize);
    getMatrix(aTemp,z);
    aTemp.downsample(aNewXSize,aNewYSize);
    for (int i = 0; i < aSize; i++)
      mData2[i+z*aSize] = aTemp.data()[i];
  }
  delete[] mData;
  mData = mData2;
  mXSize = aNewXSize;
  mYSize = aNewYSize;
}

// upsample
template <class T>
void CTensor<T>::upsample(int aNewXSize, int aNewYSize) {
  T* mData2 = new T[aNewXSize*aNewYSize*mZSize];
  int aSize = aNewXSize*aNewYSize;
  for (int z = 0; z < mZSize; z++) {
    CMatrix<T> aTemp(mXSize,mYSize);
    getMatrix(aTemp,z);
    aTemp.upsample(aNewXSize,aNewYSize);
    for (int i = 0; i < aSize; i++)
      mData2[i+z*aSize] = aTemp.data()[i];
  }
  delete[] mData;
  mData = mData2;
  mXSize = aNewXSize;
  mYSize = aNewYSize;
}

// upsampleBilinear
template <class T>
void CTensor<T>::upsampleBilinear(int aNewXSize, int aNewYSize) {
  T* mData2 = new T[aNewXSize*aNewYSize*mZSize];
  int aSize = aNewXSize*aNewYSize;
  for (int z = 0; z < mZSize; z++) {
    CMatrix<T> aTemp(mXSize,mYSize);
    getMatrix(aTemp,z);
    aTemp.upsampleBilinear(aNewXSize,aNewYSize);
    for (int i = 0; i < aSize; i++)
      mData2[i+z*aSize] = aTemp.data()[i];
  }
  delete[] mData;
  mData = mData2;
  mXSize = aNewXSize;
  mYSize = aNewYSize;
}

// fill
template <class T>
void CTensor<T>::fill(const T aValue) {
  int wholeSize = mXSize*mYSize*mZSize;
  for (register int i = 0; i < wholeSize; i++)
    mData[i] = aValue;
}

// fillCircle
template <class T>
void CTensor<T>::fillCircle(const T aValue1, const T aValue2, const T aValue3, int x, int y, int r) {
  float n=0, invradius=1/(float)r;
  int dx=0,dy=r;

  while (dx<=dy)
  {
    for(int i=dy;i>=dx;i--)
    {
      operator()(x+dx, y+i, 0) = aValue1;
      operator()(x-dx, y+i, 0) = aValue1;
      operator()(x+dx, y-i, 0) = aValue1;
      operator()(x-dx, y-i, 0) = aValue1;
      operator()(x+i, y+dx, 0) = aValue1;
      operator()(x-i, y+dx, 0) = aValue1;
      operator()(x+i, y-dx, 0) = aValue1;
      operator()(x-i, y-dx, 0) = aValue1;

      operator()(x+dx, y+i, 1) = aValue2;
      operator()(x-dx, y+i, 1) = aValue2;
      operator()(x+dx, y-i, 1) = aValue2;
      operator()(x-dx, y-i, 1) = aValue2;
      operator()(x+i, y+dx, 1) = aValue2;
      operator()(x-i, y+dx, 1) = aValue2;
      operator()(x+i, y-dx, 1) = aValue2;
      operator()(x-i, y-dx, 1) = aValue2;

      operator()(x+dx, y+i, 2) = aValue3;
      operator()(x-dx, y+i, 2) = aValue3;
      operator()(x+dx, y-i, 2) = aValue3;
      operator()(x-dx, y-i, 2) = aValue3;
      operator()(x+i, y+dx, 2) = aValue3;
      operator()(x-i, y+dx, 2) = aValue3;
      operator()(x+i, y-dx, 2) = aValue3;
      operator()(x-i, y-dx, 2) = aValue3;
    }
    dx++;
    n+=invradius;
    dy=(int)(r * sin(acos(n))+0.5);
  }
}

// cut
template <class T>
void CTensor<T>::cut(CTensor<T>& aResult, int x1, int y1, int z1, int x2, int y2, int z2) {
  aResult.mXSize = x2-x1+1;
  aResult.mYSize = y2-y1+1;
  aResult.mZSize = z2-z1+1;
  delete[] aResult.mData;
  aResult.mData = new T[aResult.mXSize*aResult.mYSize*aResult.mZSize];
  for (int z = z1; z <= z2; z++)
    for (int y = y1; y <= y2; y++)
      for (int x = x1; x <= x2; x++)
        aResult(x-x1,y-y1,z-z1) = operator()(x,y,z);
}

// cut
template <class T>
void CTensor<T>::cut(CMatrix<T>& aResult, int x1, int y1, int x2, int y2, int z) {
  aResult.setSize(x2-x1+1, y2-y1+1);
  for (int y = y1; y <= y2; y++)
    for (int x = x1; x <= x2; x++)
      aResult(x-x1,y-y1) = operator()(x,y,z);
}

// paste
template <class T>
void CTensor<T>::paste(CTensor<T>& aCopyFrom, int ax, int ay, int az) {
  for (int z = 0; z < aCopyFrom.zSize(); z++)
    for (int y = 0; y < aCopyFrom.ySize(); y++)
      for (int x = 0; x < aCopyFrom.xSize(); x++)
        operator()(ax+x,ay+y,az+z) = aCopyFrom(x,y,z);
}

// mirrorLayers
template <class T>
void CTensor<T>::mirrorLayers(int aFrom, int aTo) {
  for (int z = 0; z < mZSize; z++) {
    int aToXIndex = mXSize-aTo-1;
    int aToYIndex = mYSize-aTo-1;
    int aFromXIndex = mXSize-aFrom-1;
    int aFromYIndex = mYSize-aFrom-1;
    for (int y = aFrom; y <= aFromYIndex; y++) {
      operator()(aTo,y,z) = operator()(aFrom,y,z);
      operator()(aToXIndex,y,z) = operator()(aFromXIndex,y,z);
    }
    for (int x = aTo; x <= aToXIndex; x++) {
      operator()(x,aTo,z) = operator()(x,aFrom,z);
      operator()(x,aToYIndex,z) = operator()(x,aFromYIndex,z);
    }
  }
}

// normalize
template <class T>
void CTensor<T>::normalizeEach(T aMin, T aMax, T aInitialMin, T aInitialMax) {
  for (int k = 0; k < mZSize; k++)
    normalize(aMin,aMax,k,aInitialMin,aInitialMax);
}

template <class T>
void CTensor<T>::normalize(T aMin, T aMax, int aChannel, T aInitialMin, T aInitialMax) {
  int aChannelSize = mXSize*mYSize;
  T aCurrentMin = aInitialMax;
  T aCurrentMax = aInitialMin;
  int aIndex = aChannelSize*aChannel;
  for (int i = 0; i < aChannelSize; i++) {
    if (mData[aIndex] > aCurrentMax) aCurrentMax = mData[aIndex];
    else if (mData[aIndex] < aCurrentMin) aCurrentMin = mData[aIndex];
    aIndex++;
  }
  T aTemp1 = aCurrentMin - aMin;
  T aTemp2 = (aCurrentMax-aCurrentMin);
  if (aTemp2 == 0) aTemp2 = 1;
  else aTemp2 = (aMax-aMin)/aTemp2;
  aIndex = aChannelSize*aChannel;
  for (int i = 0; i < aChannelSize; i++) {
    mData[aIndex] -= aTemp1;
    mData[aIndex] *= aTemp2;
    aIndex++;
  }
}

// drawLine
template <class T>
void CTensor<T>::drawLine(int dStartX, int dStartY, int dEndX, int dEndY, T aValue1, T aValue2, T aValue3) {
  int aOffset1 = mXSize*mYSize;
  int aOffset2 = 2*aOffset1;
	// vertical line
	if (dStartX == dEndX) {
    if (dStartX < 0 || dStartX >= mXSize)	return;
		int x = dStartX;
		if (dStartY < dEndY) {
			for (int y = dStartY; y <= dEndY; y++)
				if (y >= 0 && y < mYSize) {
          mData[x+y*mXSize] = aValue1;
          mData[x+y*mXSize+aOffset1] = aValue2;
          mData[x+y*mXSize+aOffset2] = aValue3;
        }
  	}
		else {
			for (int y = dStartY; y >= dEndY; y--)
				if (y >= 0 && y < mYSize) {
          mData[x+y*mXSize] = aValue1;
          mData[x+y*mXSize+aOffset1] = aValue2;
          mData[x+y*mXSize+aOffset2] = aValue3;
        }
    }
    return;
  }
	// horizontal line
	if (dStartY == dEndY) {
    if (dStartY < 0 || dStartY >= mYSize) return;
 		int y = dStartY;
		if (dStartX < dEndX) {
			for (int x = dStartX; x <= dEndX; x++)
				if (x >= 0 && x < mXSize) {
          mData[x+y*mXSize] = aValue1;
          mData[x+y*mXSize+aOffset1] = aValue2;
          mData[x+y*mXSize+aOffset2] = aValue3;
        }
  	}
		else {
			for (int x = dStartX; x >= dEndX; x--)
				if (x >= 0 && x < mXSize) {
          mData[x+y*mXSize] = aValue1;
          mData[x+y*mXSize+aOffset1] = aValue2;
          mData[x+y*mXSize+aOffset2] = aValue3;
        }
    }
    return;
  }
  float m = float(dStartY - dEndY) / float(dStartX - dEndX);
  float invm = 1.0/m;
  if (fabs(m) > 1.0) {
    if (dEndY > dStartY) {
      for (int y = dStartY; y <= dEndY; y++) {
        int x = (int)(0.5+dStartX+(y-dStartY)*invm);
        if (x >= 0 && x < mXSize &&	y >= 0 && y < mYSize) {
          mData[x+y*mXSize] = aValue1;
          mData[x+y*mXSize+aOffset1] = aValue2;
          mData[x+y*mXSize+aOffset2] = aValue3;
        }
      }
    }
    else {
      for (int y = dStartY; y >= dEndY; y--) {
        int x = (int)(0.5+dStartX+(y-dStartY)*invm);
        if (x >= 0 && x < mXSize &&	y >= 0 && y < mYSize) {
          mData[x+y*mXSize] = aValue1;
          mData[x+y*mXSize+aOffset1] = aValue2;
          mData[x+y*mXSize+aOffset2] = aValue3;
        }
      }
    }
  }
  else {
    if (dEndX > dStartX) {
      for (int x = dStartX; x <= dEndX; x++) {
        int y = (int)(0.5+dStartY+(x-dStartX)*m);
        if (x >= 0 && x < mXSize &&	y >= 0 && y < mYSize) {
          mData[x+y*mXSize] = aValue1;
          mData[x+y*mXSize+aOffset1] = aValue2;
          mData[x+y*mXSize+aOffset2] = aValue3;
        }
      }
    }
    else {
      for (int x = dStartX; x >= dEndX; x--) {
        int y = (int)(0.5+dStartY+(x-dStartX)*m);
        if (x >= 0 && x < mXSize &&	y >= 0 && y < mYSize) {
          mData[x+y*mXSize] = aValue1;
          mData[x+y*mXSize+aOffset1] = aValue2;
          mData[x+y*mXSize+aOffset2] = aValue3;
        }
      }
    }
  }
}

template <class T>
void CTensor<T>::normalize(T aMin, T aMax, T aInitialMin, T aInitialMax) {
  int aSize = mXSize*mYSize*mZSize;
  T aCurrentMin = aInitialMax;
  T aCurrentMax = aInitialMin;
  for (int i = 0; i < aSize; i++) {
    if (mData[i] > aCurrentMax) aCurrentMax = mData[i];
    else if (mData[i] < aCurrentMin) aCurrentMin = mData[i];
  }
  T aTemp1 = aCurrentMin - aMin;
  T aTemp2 = (aCurrentMax-aCurrentMin);
  if (aTemp2 == 0) aTemp2 = 1;
  else aTemp2 = (aMax-aMin)/aTemp2;
  for (int i = 0; i < aSize; i++) {
    mData[i] -= aTemp1;
    mData[i] *= aTemp2;
  }
}

// readFromMathematicaFile
template <class T>
void CTensor<T>::readFromMathematicaFile(const char* aFilename) {
  using namespace std;
  // Read the whole file and store data in aData
  // Ignore blanks, tabs and lines
  // Also ignore Mathematica comments (* ... *)
  ifstream aStream(aFilename);
  string aData;
  char aChar;
  bool aBracketFound = false;
  bool aStarFound = false;
  bool aCommentFound = false;
  while (aStream.get(aChar))
    if (aChar != ' ' && aChar != '\t' && aChar != '\n') {
      if (aCommentFound) {
        if (!aStarFound && aChar == '*') aStarFound = true;
        else {
          if (aStarFound && aChar == ')') aCommentFound = false;
          aStarFound = false;
        }
      }
      else {
        if (!aBracketFound && aChar == '(') aBracketFound = true;
        else {
          if (aBracketFound && aChar == '*') aCommentFound = true;
          else aData += aChar;
          aBracketFound = false;
        }
      }
    }
  // Count the number of braces and double braces to figure out z- and y-Size of tensor
  int aDoubleBraceCount = 0;
  int aBraceCount = 0;
  int aPos = 0;
  while ((aPos = aData.find_first_of('{',aPos)+1) > 0) {
    aBraceCount++;
    if (aData[aPos] == '{' && aData[aPos+1] != '{') aDoubleBraceCount++;
  }
  // Count the number of commas in the first section to figure out xSize of tensor
  int aCommaCount = 0;
  aPos = 0;
  while (aData[aPos] != '}') {
    if (aData[aPos] == ',') aCommaCount++;
    aPos++;
  }
  // Adapt size of tensor
  if (mData != 0) delete[] mData;
  mXSize = aCommaCount+1;
  mYSize = (aBraceCount-1-aDoubleBraceCount) / aDoubleBraceCount;
  mZSize = aDoubleBraceCount;
  mData = new T[mXSize*mYSize*mZSize];
  // Analyse file ---------------
  aPos = 0;
  if (aData[aPos] != '{') throw EInvalidFileFormat("Mathematica");
  aPos++;
  for (int z = 0; z < mZSize; z++) {
    if (aData[aPos] != '{') throw EInvalidFileFormat("Mathematica");
    aPos++;
    for (int y = 0; y < mYSize; y++) {
      if (aData[aPos] != '{') throw EInvalidFileFormat("Mathematica");
      aPos++;
      for (int x = 0; x < mXSize; x++) {
        int oldPos = aPos;
        if (x+1 < mXSize) aPos = aData.find_first_of(',',aPos);
        else aPos = aData.find_first_of('}',aPos);
        #ifdef GNU_COMPILER
        string s = aData.substr(oldPos,aPos-oldPos);
        istrstream is(s.c_str());
        #else
        string s = aData.substr(oldPos,aPos-oldPos);
        istringstream is(s);
        #endif
        T aItem;
        is >> aItem;
        operator()(x,y,z) = aItem;
        aPos++;
      }
      if (y+1 < mYSize) {
        if (aData[aPos] != ',') throw EInvalidFileFormat("Mathematica");
        aPos++;
        while (aData[aPos] != '{')
          aPos++;
      }
    }
    aPos++;
    if (z+1 < mZSize) {
      if (aData[aPos] != ',') throw EInvalidFileFormat("Mathematica");
      aPos++;
      while (aData[aPos] != '{')
        aPos++;
    }
  }
}

// writeToMathematicaFile
template <class T>
void CTensor<T>::writeToMathematicaFile(const char* aFilename) {
  using namespace std;
  ofstream aStream(aFilename);
  aStream << '{';
  for (int z = 0; z < mZSize; z++) {
    aStream << '{';
    for (int y = 0; y < mYSize; y++) {
      aStream << '{';
      for (int x = 0; x < mXSize; x++) {
        aStream << operator()(x,y,z);
        if (x+1 < mXSize) aStream << ',';
      }
      aStream << '}';
      if (y+1 < mYSize) aStream << ",\n";
    }
    aStream << '}';
    if (z+1 < mZSize) aStream << ",\n";
  }
  aStream << '}';
}

// readFromIMFile
template <class T>
void CTensor<T>::readFromIMFile(const char* aFilename) {
  FILE *aStream;
  aStream = fopen(aFilename,"rb");
  // Read image data
  for (int i = 0; i < mXSize*mYSize*mZSize; i++)
    mData[i] = getc(aStream);
  fclose(aStream);
}

// writeToIMFile
template <class T>
void CTensor<T>::writeToIMFile(const char *aFilename) {
  FILE *aStream;
  aStream = fopen(aFilename,"wb");
  // write data
  for (int i = 0; i < mXSize*mYSize*mZSize; i++) {
    char dummy = (char)mData[i];
    fwrite(&dummy,1,1,aStream);
  }
  fclose(aStream);
}

// readFromPGM
template <class T>
void CTensor<T>::readFromPGM(const char* aFilename) {
  FILE *aStream;
  aStream = fopen(aFilename,"rb");
  int dummy;
  // Find beginning of file (P5)
  while (getc(aStream) != 'P');
  if (getc(aStream) != '5') throw EInvalidFileFormat("PGM");
  while (getc(aStream) != '\n');
  // Remove comments and empty lines
  dummy = getc(aStream);
  while (dummy == '#') {
    while (getc(aStream) != '\n');
    dummy = getc(aStream);
  }
  while (dummy == '\n')
    dummy = getc(aStream); 
  // Read image size
  mXSize = dummy-48;
  while ((dummy = getc(aStream)) >= 48 && dummy < 58)
    mXSize = 10*mXSize+dummy-48;
  while ((dummy = getc(aStream)) < 48 || dummy >= 58);
  mYSize = dummy-48;
  while ((dummy = getc(aStream)) >= 48 && dummy < 58)
    mYSize = 10*mYSize+dummy-48;
  mZSize = 1;
  if (dummy != '\n') while (getc(aStream) != '\n');
  while (getc(aStream) != '\n');
  // Adjust size of data structure
  delete[] mData;
  mData = new T[mXSize*mYSize];
  // Read image data
  for (int i = 0; i < mXSize*mYSize; i++)
    mData[i] = getc(aStream);
  fclose(aStream);
}

// writeToPGM
template <class T>
void CTensor<T>::writeToPGM(const char* aFilename) {
  int rows = (int)floor(sqrt((float)mZSize));
  int cols = (int)ceil(mZSize*1.0/rows);
  FILE* outimage = fopen(aFilename, "wb");
  fprintf(outimage, "P5 \n");
  fprintf(outimage, "%ld %ld \n255\n", cols*mXSize,rows*mYSize);
  for (int r = 0; r < rows; r++)
    for (int y = 0; y < mYSize; y++)
      for (int c = 0; c < cols; c++)
        for (int x = 0; x < mXSize; x++) {
          unsigned char aHelp;
          if (r*cols+c >= mZSize) aHelp = 0;
          else aHelp = (unsigned char)operator()(x,y,r*cols+c);
          fwrite (&aHelp, sizeof(unsigned char), 1, outimage);
        }
  fclose(outimage);
}

// readFromPNG
template <class T>
void CTensor<T>::readFromPNG(const char* aFilename) {
  png_byte header[8];
  FILE *f = fopen(aFilename, "rb");
  if(!f)
    fprintf(stderr, "error: file %s not found\n", aFilename);
  fread(header, 1, 8, f);
  if(png_sig_cmp(header, 0, 8))
    throw EInvalidFileFormat("PNG");

  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if(setjmp(png_jmpbuf(png_ptr)))
    return;
  png_init_io(png_ptr, f);
  png_set_sig_bytes(png_ptr, 8);
  png_read_info(png_ptr, info_ptr);

  png_uint_32 width, height;
  int bit_depth, color_type;
  png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type, NULL, NULL, NULL);


  if(color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_expand(png_ptr);
  if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand(png_ptr);
  if(png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
    png_set_expand(png_ptr);
  if(bit_depth == 16)
    png_set_strip_16(png_ptr);

  png_read_update_info(png_ptr, info_ptr);

  int rowbytes = png_get_rowbytes(png_ptr, info_ptr);

  mXSize = width;
  mYSize = height;
  mZSize = (int) png_get_channels(png_ptr, info_ptr);

  delete[] mData;
  mData = new T[mXSize*mYSize*mZSize];

  png_bytep row = new png_byte[rowbytes];
  for(int y = 0; y < mYSize; y++) {
    png_read_row(png_ptr, row, 0);
    for(int x = 0; x < mXSize; x++) {
      for(int z = 0; z < mZSize; z++)
        mData[(y * mXSize + x) + z * mXSize * mYSize] = row[x * mZSize + z];
    }
  }
  delete[] row;

  png_read_end(png_ptr, 0);
  png_destroy_read_struct(&png_ptr, &info_ptr, 0);

  fclose(f);
}

// writeToPNG
template <class T>
void CTensor<T>::writeToPNG(const char* aFilename) {
  FILE *f;
  f = fopen(aFilename,"wb");
  
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);  
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if(setjmp(png_jmpbuf(png_ptr)))
    return;
  png_init_io(png_ptr, f);

  int color_type;
  int channels = mZSize <= 4 ? mZSize : 4;
  switch(channels) {
  case 1:
    color_type = PNG_COLOR_TYPE_GRAY;
    break;
  case 2:
    color_type = PNG_COLOR_TYPE_GRAY_ALPHA;
    break;
  case 3:
    color_type = PNG_COLOR_TYPE_RGB;
    break;
  case 4:
    color_type = PNG_COLOR_TYPE_RGB_ALPHA;
    break;
  }

  png_set_IHDR(png_ptr, info_ptr, mXSize, mYSize,
               8, color_type, PNG_INTERLACE_NONE,
               PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
  png_write_info(png_ptr, info_ptr);

  png_bytep row = new png_byte[mXSize * mZSize];

  for(int y = 0; y < mYSize; y++) {
    for(int x = 0; x < mXSize; x++) {
      for(int z = 0; z < channels; z++)
        row[x * channels + z] = (unsigned char)operator()(x,y,z);
    }
    png_write_row(png_ptr, row);
  }

  delete[] row;

  png_write_end(png_ptr, info_ptr);
  
  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(f);
}

// readFromPNG
template <class T>
void CTensor<T>::readFromPNGflip(const char* aFilename) {
  png_byte header[8];
  FILE *f = fopen(aFilename, "rb");
  if(!f)
    fprintf(stderr, "error: file %s not found\n", aFilename);
  fread(header, 1, 8, f);
  if(png_sig_cmp(header, 0, 8))
    throw EInvalidFileFormat("PNG");

  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if(setjmp(png_jmpbuf(png_ptr)))
    return;
  png_init_io(png_ptr, f);
  png_set_sig_bytes(png_ptr, 8);
  png_read_info(png_ptr, info_ptr);

  png_uint_32 width, height;
  int bit_depth, color_type;
  png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type, NULL, NULL, NULL);


  if(color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_expand(png_ptr);
  if(color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand(png_ptr);
  if(png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
    png_set_expand(png_ptr);
  if(bit_depth == 16)
    png_set_strip_16(png_ptr);

  png_read_update_info(png_ptr, info_ptr);

  int rowbytes = png_get_rowbytes(png_ptr, info_ptr);

  mXSize = width;
  mYSize = height;
  mZSize = (int) png_get_channels(png_ptr, info_ptr);

  delete[] mData;
  mData = new T[mXSize*mYSize*mZSize];

  png_bytep row = new png_byte[rowbytes];
  for(int y = mYSize-1; y >=0; y--) {
    png_read_row(png_ptr, row, 0);
    for(int x = 0; x < mXSize; x++) {
      for(int z = 0; z < mZSize; z++)
        mData[(y * mXSize + x) + z * mXSize * mYSize] = row[x * mZSize + z];
    }
  }
  delete[] row;

  png_read_end(png_ptr, 0);
  png_destroy_read_struct(&png_ptr, &info_ptr, 0);

  fclose(f);
}

// writeToPNG
template <class T>
void CTensor<T>::writeToPNGflip(const char* aFilename) {
  FILE *f;
  f = fopen(aFilename,"wb");
  
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);  
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if(setjmp(png_jmpbuf(png_ptr)))
    return;
  png_init_io(png_ptr, f);

  int color_type;
  int channels = mZSize <= 4 ? mZSize : 4;
  switch(channels) {
  case 1:
    color_type = PNG_COLOR_TYPE_GRAY;
    break;
  case 2:
    color_type = PNG_COLOR_TYPE_GRAY_ALPHA;
    break;
  case 3:
    color_type = PNG_COLOR_TYPE_RGB;
    break;
  case 4:
    color_type = PNG_COLOR_TYPE_RGB_ALPHA;
    break;
  }

  png_set_IHDR(png_ptr, info_ptr, mXSize, mYSize,
               8, color_type, PNG_INTERLACE_NONE,
               PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
  png_write_info(png_ptr, info_ptr);

  png_bytep row = new png_byte[mXSize * mZSize];

  for(int y = mYSize-1; y >=0; y--) {
    for(int x = 0; x < mXSize; x++) {
      for(int z = 0; z < channels; z++)
        row[x * channels + z] = (unsigned char)operator()(x,y,z);
    }
    png_write_row(png_ptr, row);
  }

  delete[] row;

  png_write_end(png_ptr, info_ptr);
  
  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(f);
}

// makeColorTensor
template <class T>
void CTensor<T>::makeColorTensor() {
  if (mZSize != 1) return;
  int aSize = mXSize*mYSize;
  int a2Size = 2*aSize;
  T* aNewData = new T[aSize*3];
  for (int i = 0; i < aSize; i++)
    aNewData[i] = aNewData[i+aSize] = aNewData[i+a2Size] = mData[i];
  mZSize = 3;
  delete[] mData;
  mData = aNewData;
}

// readFromPPM
template <class T>
void CTensor<T>::readFromPPM(const char* aFilename) {
  FILE *aStream;
  aStream = fopen(aFilename,"rb");
  if (aStream == 0) std::cerr << "File not found: " << aFilename << std::endl;
  int dummy;
  // Find beginning of file (P6)
  while (getc(aStream) != 'P');
  if (getc(aStream) != '6') throw EInvalidFileFormat("PPM");
  while (getc(aStream) != '\n');
  // Remove comments and empty lines
  dummy = getc(aStream);
  while (dummy == '#') {
    while (getc(aStream) != '\n');
    dummy = getc(aStream);
  }
  while (dummy == '\n')
    dummy = getc(aStream); 
  // Read image size
  mXSize = dummy-48;
  while ((dummy = getc(aStream)) >= 48 && dummy < 58)
    mXSize = 10*mXSize+dummy-48;
  while ((dummy = getc(aStream)) < 48 || dummy >= 58);
  mYSize = dummy-48;
  while ((dummy = getc(aStream)) >= 48 && dummy < 58)
    mYSize = 10*mYSize+dummy-48;
  mZSize = 3;
  if (dummy != '\n') while (getc(aStream) != '\n');
  while (getc(aStream) != '\n');
  // Adjust size of data structure
  delete[] mData;
  mData = new T[mXSize*mYSize*3];
  // Read image data
  int aSize = mXSize*mYSize;
  int aSizeTwice = aSize+aSize;
  for (int i = 0; i < aSize; i++) {
    mData[i] = getc(aStream);
    mData[i+aSize] = getc(aStream);
    mData[i+aSizeTwice] = getc(aStream);
  }
  fclose(aStream);
}

// writeToPPM
template <class T>
void CTensor<T>::writeToPPM(const char* aFilename) {
  FILE* outimage = fopen(aFilename, "wb");
  fprintf(outimage, "P6 \n");
  fprintf(outimage, "%ld %ld \n255\n", mXSize,mYSize);
  for (int y = 0; y < mYSize; y++)
    for (int x = 0; x < mXSize; x++) {
      unsigned char aHelp = (unsigned char)operator()(x,y,0);
      fwrite (&aHelp, sizeof(unsigned char), 1, outimage);
      aHelp = (unsigned char)operator()(x,y,1);
      fwrite (&aHelp, sizeof(unsigned char), 1, outimage);
      aHelp = (unsigned char)operator()(x,y,2);
      fwrite (&aHelp, sizeof(unsigned char), 1, outimage);
    }
  fclose(outimage);
}

// readFromPDM
template <class T>
void CTensor<T>::readFromPDM(const char* aFilename) {
  std::ifstream aStream(aFilename);
  std::string s;
  // Read header
  aStream >> s;
  if (s != "P9") throw EInvalidFileFormat("PDM");
  char aFeatureType;
  aStream >> aFeatureType;
  aStream >> s;
  aStream >> mXSize;
  aStream >> mYSize;
  aStream >> mZSize;
  aStream >> s;
  // Adjust size of data structure
  delete[] mData;
  mData = new T[mXSize*mYSize*mZSize];
  // Read data
  for (int i = 0; i < mXSize*mYSize*mZSize; i++)
    aStream >> mData[i];
}

// writeToPDM
template <class T>
void CTensor<T>::writeToPDM(const char* aFilename, char aFeatureType) {
  std::ofstream aStream(aFilename);
  // write header
  aStream << "P9" << std::endl;
  aStream << aFeatureType << "SS" << std::endl;
  aStream << mZSize << ' ' << mYSize << ' ' << mXSize << std::endl;
  aStream << "F" << std::endl;
  // write data
  for (int i = 0; i < mXSize*mYSize*mZSize; i++) {
    aStream << mData[i];
    if (i % 8 == 0) aStream << std::endl;
    else aStream << ' ';
  }
}

// operator ()
template <class T>
inline T& CTensor<T>::operator()(const int ax, const int ay, const int az) const {
  #ifdef _DEBUG
    if (ax >= mXSize || ay >= mYSize || az >= mZSize || ax < 0 || ay < 0 || az < 0) {
      std::ofstream aStream("error.txt");
      aStream << "Exception ETensorRangeOverflow: x = " << ax << ", y = " << ay << ", z = " << az << std::endl;
      aStream << "x = " << mXSize << ", y = " << mYSize << ", z = " << mZSize << std::endl;
      aStream.close();
      throw ETensorRangeOverflow(ax,ay,az);
    }
  #endif
  return mData[mXSize*(mYSize*az+ay)+ax];
}

template <class T>
CVector<T> CTensor<T>::operator()(const float ax, const float ay) const {
  CVector<T> aResult(mZSize);
  int x1 = (int)ax;
  int y1 = (int)ay;
  int x2 = x1+1;
  int y2 = y1+1;
  #ifdef _DEBUG
  if (x2 >= mXSize || y2 >= mYSize || x1 < 0 || y1 < 0) throw ETensorRangeOverflow(ax,ay,0);
  #endif
  float alphaX = ax-x1; float alphaXTrans = 1.0-alphaX;
  float alphaY = ay-y1; float alphaYTrans = 1.0-alphaY;
  for (int k = 0; k < mZSize; k++) {
    float a = alphaXTrans*operator()(x1,y1,k)+alphaX*operator()(x2,y1,k);
    float b = alphaXTrans*operator()(x1,y2,k)+alphaX*operator()(x2,y2,k);
    aResult(k) = alphaYTrans*a+alphaY*b;
  }
  return aResult;
}

// operator =
template <class T>
inline CTensor<T>& CTensor<T>::operator=(const T aValue) {
  fill(aValue);
  return *this;
}

template <class T>
CTensor<T>& CTensor<T>::operator=(const CTensor<T>& aCopyFrom) {
  if (this != &aCopyFrom) {
    delete[] mData;
    mXSize = aCopyFrom.mXSize;
    mYSize = aCopyFrom.mYSize;
    mZSize = aCopyFrom.mZSize;
    int wholeSize = mXSize*mYSize*mZSize;
    mData = new T[wholeSize];
    for (register int i = 0; i < wholeSize; i++)
      mData[i] = aCopyFrom.mData[i];
  }
  return *this;
}

// operator +=
template <class T>
CTensor<T>& CTensor<T>::operator+=(const CTensor<T>& aTensor) {
  #ifdef _DEBUG
  if (mXSize != aTensor.mXSize || mYSize != aTensor.mYSize || mZSize != aTensor.mZSize)
    throw ETensorIncompatibleSize(mXSize,mYSize,mZSize);
  #endif
  int wholeSize = size();
  for (int i = 0; i < wholeSize; i++)
    mData[i] += aTensor.mData[i];
  return *this;
}

// operator +=
template <class T>
CTensor<T>& CTensor<T>::operator+=(const T aValue) {
  int wholeSize = mXSize*mYSize*mZSize;
  for (int i = 0; i < wholeSize; i++)
    mData[i] += aValue;
  return *this;
}

// operator *=
template <class T>
CTensor<T>& CTensor<T>::operator*=(const T aValue) {
  int wholeSize = mXSize*mYSize*mZSize;
  for (int i = 0; i < wholeSize; i++)
    mData[i] *= aValue;
  return *this;
}

// min
template <class T>
T CTensor<T>::min() const {
  T aMin = mData[0];
  int aSize = mXSize*mYSize*mZSize;
  for (int i = 1; i < aSize; i++)
    if (mData[i] < aMin) aMin = mData[i];
  return aMin;
}

// max
template <class T>
T CTensor<T>::max() const {
  T aMax = mData[0];
  int aSize = mXSize*mYSize*mZSize;
  for (int i = 1; i < aSize; i++)
    if (mData[i] > aMax) aMax = mData[i];
  return aMax;
}

// avg
template <class T>
T CTensor<T>::avg() const {
  T aAvg = 0;
  for (int z = 0; z < mZSize; z++)
    aAvg += avg(z);
  return aAvg/mZSize;
}

template <class T>
T CTensor<T>::avg(int az) const {
  T aAvg = 0;
  int aSize = mXSize*mYSize;
  int aTemp = (az+1)*aSize;
  for (int i = az*aSize; i < aTemp; i++) 
    aAvg += mData[i];
  return aAvg/aSize;
}

// xSize
template <class T>
inline int CTensor<T>::xSize() const {
  return mXSize;
}

// ySize
template <class T>
inline int CTensor<T>::ySize() const {
  return mYSize;
}

// zSize
template <class T>
inline int CTensor<T>::zSize() const {
  return mZSize;
}

// size
template <class T>
inline int CTensor<T>::size() const {
  return mXSize*mYSize*mZSize;
}

// getMatrix
template <class T>
CMatrix<T> CTensor<T>::getMatrix(const int az) const {
  CMatrix<T> aTemp(mXSize,mYSize);
  int aMatrixSize = mXSize*mYSize;
  int aOffset = az*aMatrixSize;
  for (int i = 0; i < aMatrixSize; i++)
    aTemp.data()[i] = mData[i+aOffset];
  return aTemp;
}

// getMatrix
template <class T>
void CTensor<T>::getMatrix(CMatrix<T>& aMatrix, const int az) const {
  if (aMatrix.xSize() != mXSize || aMatrix.ySize() != mYSize)
    throw ETensorIncompatibleSize(aMatrix.xSize(),aMatrix.ySize(),mXSize,mYSize);
  int aMatrixSize = mXSize*mYSize;
  int aOffset = az*aMatrixSize;
  for (int i = 0; i < aMatrixSize; i++)
    aMatrix.data()[i] = mData[i+aOffset];
}

// putMatrix
template <class T>
void CTensor<T>::putMatrix(CMatrix<T>& aMatrix, const int az) {
  if (aMatrix.xSize() != mXSize || aMatrix.ySize() != mYSize)
    throw ETensorIncompatibleSize(aMatrix.xSize(),aMatrix.ySize(),mXSize,mYSize);
  int aMatrixSize = mXSize*mYSize;
  int aOffset = az*aMatrixSize;
  for (int i = 0; i < aMatrixSize; i++)
    mData[i+aOffset] = aMatrix.data()[i];
}

// data()
template <class T>
inline T* CTensor<T>::data() const {
  return mData;
}

// N O N - M E M B E R  F U N C T I O N S --------------------------------------

// operator <<
template <class T>
std::ostream& operator<<(std::ostream& aStream, const CTensor<T>& aTensor) {
  for (int z = 0; z < aTensor.zSize(); z++) {
    for (int y = 0; y < aTensor.ySize(); y++) {
      for (int x = 0; x < aTensor.xSize(); x++)
        aStream << aTensor(x,y,z) << ' ';
      aStream << std::endl;
    }
    aStream << std::endl;
  }
  return aStream;
}

#endif
