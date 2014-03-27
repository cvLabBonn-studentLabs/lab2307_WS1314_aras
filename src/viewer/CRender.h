// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

#ifndef CRenderH
#define CRenderH


#include <vector>

#include <cxcore.h>

#include "fbo.h"
#include "CTMesh.h"

class CRender {
public:
  // constructor
  CRender(int aWidth, int aHeight);
  // destructor
  ~CRender() {
    delete[] image1; 
    delete[] image1F; 
    delete[] image3;
    delete[] projection;
    delete[] modelview;
    delete[] viewport;
  };
  // load camera
  void loadCameraConfig(const char* filename);
  void loadImage3(IplImage* img);

  void visualize(CMesh* mesh, IplImage* img, int bp = -1);
  
  void getDepth(CMesh* mesh, IplImage* img, int bp = -1);
  void getSilhouette(CMesh* mesh, IplImage* img, int bp = -1);

  // Fast projection of a 3-D point to the image plane
  inline void projectPoint(const CVector<float>& v3D, int& x, int& y) const;
  inline void projectPoint(float X, float Y, float Z, int& x, int& y) const;
  inline void projectNormal(const CVector<float>& v3Dn, float& x, float& y) const;
  void projectPoint(const CVector<float>& v3D, double* x, double* y, double* z) const;

private:

  void setIpl3(IplImage* img);
  void getIpl3(IplImage* img);
  void getIpl1F(IplImage* img);
  void getIpl1(IplImage* img);
  void reset3();

  void drawCoordinate() const;
  void render2D() const;
  void activate2Dmode() const;
  void deactivate2Dmode() const;

  // Basic data image size
  int mW, mH;

  // Projection Matrix 
  CMatrix<float> P;
  // Matrices OpenGL
  double* projection;
  double* modelview;
  int* viewport;

  // Framebuffer object for rendering
  fbo FBO;

  // store RGB image as texture
  unsigned int image_texture;
  unsigned char* image1;
  GLfloat* image1F;
  unsigned char* image3;
  //unsigned char* image_rgb;

};

// projectPoint
void CRender::projectPoint(const CVector<float>& v3D, int& x, int& y) const {
  float hx = P.data()[0]*v3D[0] + P.data()[1]*v3D[1] + P.data()[2]*v3D[2]  + P.data()[3];
  float hy = P.data()[4]*v3D[0] + P.data()[5]*v3D[1] + P.data()[6]*v3D[2]  + P.data()[7];
  float hz = P.data()[8]*v3D[0] + P.data()[9]*v3D[1] + P.data()[10]*v3D[2] + P.data()[11];
  
  float invhz = 1.0/hz;
  x = (int)(hx*invhz);
  y = (int)(hy*invhz);
}

void CRender::projectPoint(float X, float Y, float Z, int& x, int& y) const {
  float hx = P.data()[0]*X + P.data()[1]*Y + P.data()[2]*Z  + P.data()[3];
  float hy = P.data()[4]*X + P.data()[5]*Y + P.data()[6]*Z  + P.data()[7];
  float hz = P.data()[8]*X + P.data()[9]*Y + P.data()[10]*Z + P.data()[11];
  
  float invhz = 1.0/hz;
  x = (int)(hx*invhz);
  y = (int)(hy*invhz);
}

// projectPoint
void CRender::projectNormal(const CVector<float>& v3Dn, float& x, float& y) const {
  x = P.data()[0]*v3Dn[0] + P.data()[1]*v3Dn[1] + P.data()[2]*v3Dn[2]  + P.data()[3];
  y = P.data()[4]*v3Dn[0] + P.data()[5]*v3Dn[1] + P.data()[6]*v3Dn[2]  + P.data()[7];
  float z = P.data()[8]*v3Dn[0] + P.data()[9]*v3Dn[1] + P.data()[10]*v3Dn[2] + P.data()[11];
  
  float invz = fabs(1.0/z);
  x *= invz;
  y *= invz;
}


#endif
