// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

// Standard header 
#include <sys/time.h>
#include <iostream>
#include <fstream>

// OpenCV header
#include <cxcore.h>
#include <cv.h>
#include <highgui.h>

// OpenGL header
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glx.h>

#include "NShow.h"
#include "CTMesh.h"
#include "CRender.h"

#include "CVector.h"
#include "CMatrix.h"
#include "NRBM.h"

#include "Transform.h"

using namespace std;

CMesh initMesh;
CMesh curMesh;
char buffer[1024];

// Set up directories for in- and output
// Read config file
void parseArg(int argc, char** argv) {

  if (argc < 3) { 
    cerr << "Usage: ./puppetModel <configfile> <dstpath>" << endl;
    exit(-1);
  }

  // read configfile
  ifstream iStream(argv[1]);
  if(iStream.is_open()) {

    iStream >> NShow::image_file;
    iStream >> NShow::image_path;
    iStream >> NShow::start_frame;
    iStream >> NShow::end_frame;
    iStream >> NShow::camera_file;
    iStream >> NShow::cameraRGB_file;
    iStream >> NShow::mesh_file;
    iStream >> NShow::est_path;
    iStream >> NShow::fileWrite_path;

    iStream.close();

    cout << "Image   : " << NShow::image_file << " " << NShow::image_path << " " << NShow::start_frame << " " << NShow::end_frame << endl;
    cout << "Camera  : " << NShow::camera_file << " " << NShow::cameraRGB_file << endl;
    cout << "Mesh    : " << NShow::mesh_file << endl;
    cout << "Estimate: " << NShow::est_path << endl;


  } else {
    cerr << "Cannot open " << argv[1] << endl;
    exit(-1);
  }

  // create output directory
  NShow::output_path = argv[2];
  sprintf(buffer,"mkdir %s",argv[2]);
  system(buffer);

}

// load depth, X, Y, Z image 
// the 32 bit float values are stored 4x8 bit color image 
bool loadFloatImage(const IplImage* out, const char* filename) {
  
  IplImage* img = 0;
  img = cvLoadImage(filename, CV_LOAD_IMAGE_GRAYSCALE);
  bool res = (img!=0);

  if(res) {

    // data for input image
    uchar* indata;
    int stepIn;
    cvGetRawData( img, (uchar**)&indata, &stepIn);
    int si = sizeof(indata[0]);
    stepIn /= si;
    
    // data for output image
    int so = sizeof(float);
    float* outdata;
    int stepOut;
    cvGetRawData( out, (uchar**)&outdata, &stepOut);
    stepOut /= so;
        
    // copy float -> uchar
    for( int y = 0; y < img->height; y++, indata += stepIn, outdata += stepOut) {
      int m = 0;
      for( int k=0; k < so; k++)
	for( int l = k; l < out->width*so; l+=so ) {
	  *((uchar*)(outdata)+l*si) = *((uchar*)(indata) + m*si);
	  m++;
	}
    }
    
    cvReleaseImage(&img);
  
  } else {
  
    cout << "Could not find " << filename << endl;
  
  }

  return res;
}

// convert 32bit float image to 8bit image
// values outside of min_d and max_d are set to the 
void depth28bit(IplImage* in, IplImage* out, float min_d, float max_d) {
  int stepIn, stepOut;
  float* dataIn;
  uchar* dataOut;
  cvGetRawData( in, (uchar**)&dataIn, &stepIn);
  cvGetRawData( out, (uchar**)&dataOut, &stepOut);
  stepIn /= sizeof(dataIn[0]);
  stepOut /= sizeof(dataOut[0]);
  
  for( int y = 0; y < in->height; y++, dataIn += stepIn, dataOut += stepOut) {
    for( int x = 0; x < in->width; x++ ) {

      // map range to 0..255
      if (dataIn[x] > max_d) 
	dataOut[x] = 0;
      else if (dataIn[x] < min_d)
	dataOut[x] = 255;
      else
	dataOut[x] = (unsigned char)((max_d - dataIn[x]) * (255.0/max_d));

    }
  }
}

// read image file names
void loadBMF(vector<string>& vImages) {
  
  // read filenames
  std::ifstream iStream(NShow::image_file.c_str());
  if(iStream.is_open()) {
    int tmp;
    iStream >> tmp;
    
    if(tmp<NShow::end_frame) {
      cerr << "Invalid end frame " << NShow::end_frame << " " << tmp << endl; exit(-1);
    }
    
    string dummy;
    for(int i=1;i<NShow::start_frame;++i)
      iStream >> dummy; 
    
    vImages.resize(NShow::end_frame-NShow::start_frame+1);
    for(int i=0;i<vImages.size();++i)
      iStream >> vImages[i]; 
    
    iStream.close();
  } else {
    cerr << "Error reading file: " << NShow::image_file << endl;
    exit(-1);
  }
  
}

// read joint parameters
void loadEstimate(vector<vector<float> >& XYZ, vector<CVector<float> >& JOINTS, vector<CMatrix<float> >& RBM) {
  
  // read XYZ coordinates 
  std::ifstream iStream( (NShow::est_path + "/absXYZ.txt").c_str() );
  if(iStream.is_open()) {

    float dummy;
    for(int i=1;i<NShow::start_frame;++i)
      for(int j=0;j<39;++j) {
	iStream >> dummy;
      }
    
    for(int i=0;i<XYZ.size();++i) {
      XYZ[i].resize(39);
      for(int j=0;j<39;++j) {
	iStream >> XYZ[i][j]; 
      }
    }
    
    iStream.close();
  } else {
    cerr << "Error reading file: " << NShow::est_path << "/absXYZ.txt" << "\n";
    exit(-1);
  }
  
  // read joint values
  iStream.open( (NShow::est_path + "/joints.txt").c_str() );
  if(iStream.is_open()) {

    float dummy;
    for(int i=1;i<NShow::start_frame;++i)
      for(int j=0;j<10;++j) {
	iStream >> dummy;
	iStream.ignore(1,'|');
      }
    
    for(int i=0;i<JOINTS.size();++i) {
      JOINTS[i].setSize(10);
      for(int j=0;j<10;++j) {
	iStream >> JOINTS[i](j); 
	iStream.ignore(1,'|');
      }
    }
    
    iStream.close();
  } else {
    cerr << "Error reading file: " << NShow::est_path << "/joints.txt" << "\n";
    exit(-1);
  }

  // read rigid body motion
  iStream.open( (NShow::est_path + "/rbm.txt").c_str() );
  if(iStream.is_open()) {

    float dummy;
    for(int i=1;i<NShow::start_frame;++i)
      for(int j=0;j<16;++j) {
	iStream >> dummy;
      }

    for(int i=0;i<RBM.size();++i) {
      RBM[i].setSize(4,4);
      for(int j=0;j<4;++j) {
	iStream >> RBM[i](0,j); 
	iStream >> RBM[i](1,j); 
	iStream >> RBM[i](2,j); 
	iStream >> RBM[i](3,j); 
      }
    }

    iStream.close();
  } else {
    cerr << "Error reading file: " << NShow::est_path << "/rbm.txt" << "\n";
    exit(-1);
  }
  
}  

#define WRITE_JOINT_FILE     1
#define SAVE_OVERLAY_IMAGE   0
#define SAVE_UNDISTORT_RGB   0
#define SAVE_UNDISTORT_DEP   0

/////////////////////////////////////////////////////////////////////
// Main                                                            //
/////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

  // ************** Init GLUT ************** //
  glutInit(&argc, argv);
  glutInitWindowSize (1, 1);
  glutCreateWindow("Test"); 
  glutHideWindow();

  if (glewInit() != GLEW_OK) {
    cerr << "Glew failed to initialize." << endl;
    exit(-1);
  }
    
  // ************** Read Input ************* //
  cout << endl << "*** ParseArg ***" << endl;
  parseArg(argc, argv);

  cout << endl << "*** InitRender ***" << endl;
  CRender renderRGB(640,480);
  renderRGB.loadCameraConfig( NShow::cameraRGB_file.c_str() );

  CRender render(176,144);
  render.loadCameraConfig( NShow::camera_file.c_str() );

  cout << endl << "*** InitMesh ***" << endl;
  if( !initMesh.readModel( NShow::mesh_file.c_str(), true) ) exit(-1);
  curMesh = initMesh;
  curMesh.setupRenderBuffers();

  cout << endl << "*** LoadCamera ***" << endl;
  Transform* TOFtrans = new Transform(176,144);
  TOFtrans->loadCameraConfig(NShow::camera_file.c_str());

  Transform* RGBtrans = new Transform(640,480);
  RGBtrans->loadCameraConfig(NShow::cameraRGB_file.c_str());

  // data structure
  IplImage* imInt   = cvCreateImage( cvSize(176,144) , IPL_DEPTH_32F , 1);
  IplImage* imInt2   = cvCreateImage( cvSize(176,144) , IPL_DEPTH_32F , 1);
  IplImage* imGray   = cvCreateImage( cvSize(176,144) , IPL_DEPTH_8U , 1);
  IplImage* imShow   = cvCreateImage( cvSize(176,144) , IPL_DEPTH_8U , 3);
  IplImage* imShowRGB   = cvCreateImage( cvSize(640,480) , IPL_DEPTH_8U , 3);

  // read image file names
  vector<string> vImages;
  loadBMF(vImages);

  // read pose estimates
  vector<vector<float> > XYZ(vImages.size());
  vector<CVector<float> > JOINTS(vImages.size());
  vector<CMatrix<float> > RBM(vImages.size());
  loadEstimate(XYZ,JOINTS,RBM);

#if WRITE_JOINT_FILE
  // file to write all joint locations
  sprintf(buffer,"%s/joints_dep.txt",NShow::output_path.c_str());
  std::ofstream dep_joint_out(buffer);
  for(int idx=0; idx<NShow::start_frame; ++idx){
    for (int k=0; k<13; ++k) {
	dep_joint_out<<0<<"\t"<<0<<"\t";    
    }
    dep_joint_out<<std::endl;
  }

  sprintf(buffer,"%s/joints_rgb_un.txt",NShow::output_path.c_str());
  std::ofstream rgb_joint_out(buffer);
  for(int idx=0; idx<NShow::start_frame; ++idx){
    for (int k=0; k<13; ++k) {
	rgb_joint_out<<0<<"\t"<<0<<"\t";    
    }
    rgb_joint_out<<std::endl;
  }
#endif 

  for(int i=0; i<vImages.size(); ++i) {
    
    // read and undistort amplitude image
    // X,Y,depth images can be read in the same way
    sprintf(buffer,("%s/" + vImages[i]).c_str(),NShow::image_path.c_str(),"depth");
    loadFloatImage(imInt, buffer);
    TOFtrans->undistort(imInt, imInt2);

    // convert to 8bit for visualization
    depth28bit(imInt2, imGray, 0.5, 3.0);
    cvMerge(imGray, imGray, imGray, NULL, imShow);

#if SAVE_UNDISTORT_DEP
    // save image
    sprintf(buffer,("%s/" + vImages[i]).c_str(),NShow::fileWrite_path.c_str(),"depth");
    cout << buffer <<  endl;
    cvSaveImage(buffer, imShow);
#endif

    // project mesh
    curMesh = initMesh;
    CVector<CMatrix<float> > M(curMesh.joints()+1);
    curMesh.angleToMatrix(RBM[i], JOINTS[i], M);
    curMesh.rigidMotion(M, JOINTS[i],true);
    curMesh.generateNormals();
    curMesh.updateVertexBuffer();
    render.loadImage3(imShow);
    render.visualize(&curMesh, imShow);

    // project joint positions
    int x1, y1;
    for (int k=0; k<XYZ[i].size(); k+=3) {
      render.projectPoint(XYZ[i][k],XYZ[i][k+1],XYZ[i][k+2],x1,y1);
      if(x1 >= 0 && y1 >= 0 && x1 < imShow->width && y1 < imShow->height) {
	cvCircle(imShow, cvPoint(x1,y1), 3, cvScalar(0,255,0),-1);
      }    
    }

#if WRITE_JOINT_FILE
    // save the position in joint file
    for (int k=0; k<XYZ[i].size(); k+=3) {
      render.projectPoint(XYZ[i][k],XYZ[i][k+1],XYZ[i][k+2],x1,y1);
      dep_joint_out<<x1<<"\t"<<y1<<"\t";    
    }
    dep_joint_out<<std::endl;
#endif

#if SAVE_OVERLAY_IMAGE
    // save image
    sprintf(buffer,"%s/checkDEP_%05d.png",NShow::output_path.c_str(),i+NShow::start_frame);
    cout << buffer <<  endl;
    cvSaveImage(buffer, imShow);
#endif 

    // read and undistort color image
    sprintf(buffer,("%s/" + vImages[i]).c_str(),NShow::image_path.c_str(),"rgb");
    IplImage* imRGB = cvLoadImage(buffer, CV_LOAD_IMAGE_COLOR);
    RGBtrans->undistort(imRGB, imShowRGB);

#if SAVE_UNDISTORT_RGB
    // save RGB image
    sprintf(buffer,("%s/" + vImages[i]).c_str(),NShow::fileWrite_path.c_str(),"rgb");
    cout << buffer <<  endl;
    cvSaveImage(buffer, imShowRGB);
#endif

    // project joint positions to RGB image
    for (int k=0; k<XYZ[i].size(); k+=3) {
      renderRGB.projectPoint(XYZ[i][k],XYZ[i][k+1],XYZ[i][k+2],x1,y1);
      if(x1 >= 0 && y1 >= 0 && x1 < imShowRGB->width && y1 < imShowRGB->height) {
	cvCircle(imShowRGB, cvPoint(x1,y1), 5, cvScalar(0,255,0),-1);
      }    
    }   
    cvReleaseImage(&imRGB);

#if WRITE_JOINT_FILE
    // save the position in joint file
    for (int k=0; k<XYZ[i].size(); k+=3) {
      renderRGB.projectPoint(XYZ[i][k],XYZ[i][k+1],XYZ[i][k+2],x1,y1);
      rgb_joint_out<<x1<<"\t"<<y1<<"\t";    
    }
    rgb_joint_out<<std::endl;
#endif

#if SAVE_OVERLAY_IMAGE
    // save RGB image
    sprintf(buffer,"%s/checkRGB_%05d.png",NShow::output_path.c_str(),i+NShow::start_frame);
    cout << buffer <<  endl;
    cvSaveImage(buffer, imShowRGB);
#endif

  }

#if WRITE_JOINT_FILE
  dep_joint_out.close();
  rgb_joint_out.close();
#endif

  cvReleaseImage(&imInt);
  cvReleaseImage(&imInt2);
  cvReleaseImage(&imShow);
  cvReleaseImage(&imShowRGB);
  cvReleaseImage(&imGray);

  return 0;
}

