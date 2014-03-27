// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ########################################################################## 

#include "CRender.h"

#include <GL/glut.h>

using namespace std;

CRender::CRender(int w, int h) {

  mW = w; mH = h;
  image1 = new unsigned char[mW * mH];
  image1F = new float[mW * mH];
  image3 = new unsigned char[mW * mH * 3];
  

  FBO.initialize(w, h);
  FBO.addColorBuffer();
  FBO.addDepthBuffer();
  FBO.checkFramebufferStatus();
  FBO.bind();

  if (!GLEW_ARB_texture_rectangle) {
    cerr << "Rectangular textures not supported" << endl;
    exit(-1);
  }

  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);

  image_texture = -1;

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glDisable(GL_LIGHTING);
  glShadeModel(GL_FLAT);

  projection = new double[16];
  modelview = new double[16];
  viewport = new int[4];
}

void CRender::loadCameraConfig(const char* filename) {
  ifstream aStream(filename);
  if(aStream.is_open()) {
    float tmp;
    
    P = CMatrix<float>(4,3,0);
    aStream>>P(0,0); //fx;
    aStream>>P(1,1); //fy;
    aStream>>P(2,0); //cx;
    aStream>>P(2,1); //cy;
    P(2,2) = 1.0;
        
    for(int i=0;i<5;++i)
      aStream >> tmp;

    float MVinv[16];

    aStream>>MVinv[0];
    aStream>>MVinv[4];
    aStream>>MVinv[8];
    aStream>>MVinv[1];
    aStream>>MVinv[5];
    aStream>>MVinv[9];
    aStream>>MVinv[2];
    aStream>>MVinv[6];
    aStream>>MVinv[10];

    aStream>>MVinv[12];
    aStream>>MVinv[13];
    aStream>>MVinv[14];

    MVinv[3]  = 0;
    MVinv[7]  = 0;
    MVinv[11] = 0;
    MVinv[15] = 1;

    aStream.close();

    // -z
    for(int i = 0; i<4; ++i)
      MVinv[i*4+2] *= -1; 


    float f = P(1,1)*2.0/(float)mH;
    float aspect = f/P(0,0)*(float)mW/2.0;
    float ClipNear =  500.0;//1000
    float ClipFar =  5000.0;//10000
    float cx = P(2,0); 
    float cy = P(2,1);
  
    float left   = aspect*ClipNear/f*2.0*(-cx/(float)mW);
    float right  = aspect*ClipNear/f*2.0*(1.0-cx/(float)mW);
    float top    = ClipNear/f*2.0*(1.0-cy/(float)mH);
    float bottom = ClipNear/f*2.0*(-cy/(float)mH);
  
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();    	
    glFrustum(left, right, bottom, top, ClipNear, ClipFar);

    glMatrixMode ( GL_MODELVIEW ); 
    glLoadIdentity ( );
    glMultMatrixf( MVinv );

    CMatrix<float> M(4,4);
    for(int i = 0; i<4; ++i)
      for(int j = 0; j<4; ++j) {
	if(j==2) 
	  M.data()[i + j*4] = -MVinv[j + i*4];
	else
	  M.data()[i + j*4] = MVinv[j + i*4];
      }
  

    cout << "K: " << P;
    cout << "M: " << M;
    P *= M;
    cout << "P: " << P << endl;

    glGetDoublev(GL_PROJECTION_MATRIX , projection);
    glGetDoublev(GL_MODELVIEW_MATRIX , modelview);
    glGetIntegerv(GL_VIEWPORT ,viewport);
    

  } else {
    cerr << "Error reading camera calibration file: " << filename << "\n";
    exit(-1);
  }
}


void CRender::loadImage3(IplImage* img) {
  if (image_texture != -1)
    glDeleteTextures(1, &image_texture);

  setIpl3(img);
	
  glGenTextures(1, &image_texture);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, image_texture);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, mW, mH, 0, GL_RGB, GL_UNSIGNED_BYTE, image3);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
}

void CRender::visualize(CMesh* mesh, IplImage* img, int bp) {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  render2D();

  glPolygonMode(GL_FRONT, GL_LINE);
 
  glColor3f(0.3,1,1);
  mesh->render(bp);

  glPolygonMode(GL_FRONT, GL_FILL);

  // debug
  //drawCoordinate();

  glFlush();

  glReadPixels(0,0, mW, mH, GL_RGB, GL_UNSIGNED_BYTE, image3);
  
  getIpl3(img);
}

void CRender::getDepth(CMesh* mesh, IplImage* img, int bp) {

  glClear(GL_DEPTH_BUFFER_BIT);
 
  mesh->render(bp);

  glFlush();

  glReadPixels(0,0, mW, mH, GL_DEPTH_COMPONENT, GL_FLOAT, image1F);
  
  getIpl1F(img);
}

void CRender::getSilhouette(CMesh* mesh, IplImage* img, int bp) {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f(1,1,1);
 
  mesh->render(bp);

  glFlush();

  glReadPixels(0,0, mW, mH, GL_LUMINANCE, GL_UNSIGNED_BYTE, image1);
  
  getIpl1(img);
}

void CRender::projectPoint(const CVector<float>& v3D, double* x, double* y, double* z) const {
  gluProject(v3D[0],v3D[1],v3D[2],modelview,projection,viewport,x,y,z);
}

void CRender::drawCoordinate() const {

  cout << "Draw Coordinate" << endl;
  glPushMatrix();
  float dx = 0, dy = 0, dz = 1500;
  glTranslatef(dx,dy,dz);
  glColor3f(0.4,0.4,0.4);
  glutWireSphere(40,5,5);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(200+dx,dy,dz);
  glColor3f(1.0,0.0,0.0);
  glutWireSphere(20,5,5);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(dx,200+dy,dz);
  glColor3f(0.0,1.0,0.0);
  glutWireSphere(20,5,5);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(dx,dy,200+dz);
  glColor3f(0.0,0.0,1.0);
  glutWireSphere(20,5,5);
  glPopMatrix();
}

void CRender::render2D() const {
  activate2Dmode();

  glEnable(GL_TEXTURE_RECTANGLE_ARB);

  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, image_texture);

  glBegin(GL_QUADS);
  glColor3f(1,1,1);
  glTexCoord2i(0, 0);
  glVertex3f  (0, 0, 0);
  glTexCoord2i(mW, 0);
  glVertex3f  (1, 0, 0);
  glTexCoord2i(mW,mH);
  glVertex3f  (1, 1, 0);
  glTexCoord2i(0, mH);
  glVertex3f  (0, 1, 0);
  glEnd();
  glDisable(GL_TEXTURE_RECTANGLE_ARB);

  deactivate2Dmode();
}

void CRender::setIpl3(IplImage* img) {
  int step;
  uchar* data;
  cvGetRawData( img, (uchar**)&data, &step);
  step /= sizeof(data[0]);

  unsigned char* p = &(image3[0]);
  for (int y=0; y<mH; ++y, data+=step) {
    for(int x=0; x<mW; ++x) {
      *(p++) = data[3*x+2];
      *(p++) = data[3*x+1];
      *(p++) = data[3*x];
    }
  }
}

void CRender::reset3() {
  unsigned char* p = &(image3[0]);
  for (int i=0; i<mH*mW*3; ++i, ++p) {
    *p = 0;
  }
}

void CRender::getIpl3(IplImage* img) {
  int step;
  uchar* data;
  cvGetRawData( img, (uchar**)&data, &step);
  step /= sizeof(data[0]);

  unsigned char* p = &(image3[0]);
  for (int y=0; y<mH; ++y, data+=step) {
    for(int x=0; x<mW; ++x) {
      data[3*x+2] = *(p++);
      data[3*x+1] = *(p++);
      data[3*x  ] = *(p++);
    }
  }
}

void CRender::getIpl1F(IplImage* img) {
  int step;
  float* data;
  cvGetRawData( img, (uchar**)&data, &step);
  step /= sizeof(data[0]);

  GLfloat* p = &(image1F[0]);
  for (int y=0; y<mH; ++y, data+=step) {
    for(int x=0; x<mW; ++x) {
      data[x] = *(p++);
    }
  }
}

void CRender::getIpl1(IplImage* img) {
  int step;
  uchar* data;
  cvGetRawData( img, (uchar**)&data, &step);
  step /= sizeof(data[0]);

  unsigned char* p = &(image1[0]);
  for (int y=0; y<mH; ++y, data+=step) {
    for(int x=0; x<mW; ++x) {
      data[x] = *(p++);
    }
  }
}

void CRender::activate2Dmode() const {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, 0, 1 );
  glDisable(GL_DEPTH_TEST);
}

void CRender::deactivate2Dmode() const {
  glEnable(GL_DEPTH_TEST);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}
