/*
 * =====================================================================================
 *
 *       Filename:  main_cvpr_data.cpp
 *
 *    Description:  plays back depth mocap data for CVPR 2010
 *        Version:  1.0
 *        Created:  6/8/2010 4:12:38 PM
 *         Author:  Varun Ganapathi (varung@gmail.com), 
 * =====================================================================================
 */
#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

namespace stanford_eval {

int             paused = false;
float           amp_lo = 4000;
float           x_angle = 0,
                y_angle = 0;
int             cur_file=0;
std::string     data_set_dir = "data/stanford/cvpr10/data";
int 			id = 0;
//extern float    color_map[][3];
//extern int      color_map_size;



float intrinsic[] = {
  2.844376,0.000000,0.000000,0.000000,
  0.000000,2.327216,0.000000,0.000000,
  0.000000,0.000000,-1.000020,-1.000000,
  0.000000,0.000000,-0.200002,0.000000 
};

struct Frame {
  static const int  R = 176;
  static const int  C = 144;
  static const int  N = R*C;
  static const int  MAX_MARKERS = 64;


  int               MAGIC_NUMBER;    // 4 byte header to detect read failure
  int               frame_number;    // 4 bytes
  long long         frame_timestamp; // 8 bytes
  float             points[N][3];    // N * 4 bytes, points in TOF frame
  unsigned short    dist[N],         // N * 1 bytes, ~ depth 
                    amp[N],          // N * 1 bytes, ~ intensity
                    conf[N];         // N * 1 bytes, ~ confidence

  struct Marker {
    int id;
    int frame;
    float x, y, z;
    float cond;
    unsigned int flag;
  };
  int                nmarkers;
  Marker             markers[MAX_MARKERS];

  // constant size per frame
  void write_constant(ostream& os){
    os.write( (char*)this, sizeof(*this)); 
  }
  void read_constant(istream& os){
    assert_sizes();
    os.read( (char*)this, sizeof(*this)); 
    if(!os) { 
      //printf("error: read %d\n", os.gcount());
    }
    assert(MAGIC_NUMBER==-1982 && "MAGIC NUMBER NOT FOUND");
  }

  // asserts that byte sizes on this computer match dataset
  void assert_sizes() {
    assert(sizeof(int) == 4);
    assert(sizeof(long long) == 8);
    assert(sizeof(unsigned short) == 2 );
    assert(sizeof(float) == 4);
  }

  Frame():MAGIC_NUMBER(-1982){}
};

//void reshape( int w, int h ){
//  glutPostRedisplay();
//}

inline float clamp( float x, float lo, float hi){
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}

Frame frame;
ifstream seq_ifs; 

//void draw( Frame& f ) {
//  glMatrixMode(GL_PROJECTION);
//  glLoadIdentity();
//  glLoadMatrixf(intrinsic);
//
//  // draw sensor
//  glEnable(GL_DEPTH_TEST);
//  glPointSize(4.0);
//  glMatrixMode(GL_MODELVIEW);
//  glLoadIdentity();
//
//  glTranslatef(0,0,-2);
//  glRotatef(y_angle, 0, 1.0, 0);
//  glRotatef(x_angle, 1.0, 0, 0);
//  glTranslatef(0,0,2);
//
//  glColor3f( .7, .3, .3 );
//  glBegin(GL_POINTS);
//
//  for (int i=0; i<f.N; ++i) {
//    float a = float(f.amp[i])/amp_lo, *p = &(f.points[i][0]);
//    a = a>1.0 ? 1.0 : a;
//    a = a<0.0 ? 0.0 : a;
//    //glColor3f( a, a, a );
//    int index = clamp(-(f.points[i][2] + 2.0), 0.0, 1.0 ) * (color_map_size-1);
//    glColor3fv( color_map[ index ] );
//    glVertex3fv(p);
//  }
//  glEnd();
//
//
//  // draw markers
//  glDisable(GL_DEPTH_TEST);
//  for(int i=0; i<f.nmarkers; ++i) {
//    if(f.markers[i].cond < 0 ) { glColor3f(1,0,0); }
//    glPushMatrix();
//    glTranslatef( f.markers[i].x, f.markers[i].y, f.markers[i].z );
//    glColor3f(0,1,0);
//    glutWireSphere(.01, 10,10);
//    stringstream labelText;
//    labelText << i;
//    glColor3f( 0, 1, 0 );
//    glRasterPos3f( 0, 0, 0 );
//    glutBitmapString( GLUT_BITMAP_HELVETICA_18, (const unsigned char*) labelText.str().c_str() ); // requires openglut or freeglut
//    glPopMatrix();
//  }
//}



bool read_frame() {
  frame.read_constant(seq_ifs);
  // open the next one
  if(!seq_ifs) {
    seq_ifs.clear();
    return false;
  }
  return true;
}

bool next_file() {
  ++id;
  char buf[1024];
  sprintf(buf, "%s/real01_%d.cal.bin", data_set_dir.c_str(), id);
  //printf("trying to open: %s\n",buf);
  seq_ifs.clear();
  if(seq_ifs.is_open()) seq_ifs.close();
  seq_ifs.open(buf, ifstream::binary);
  if(!seq_ifs) {
    //printf("ERROR: could not open: %s\n", buf);
    return false;
  }
  read_frame();
  return true;
}

void idle() {
  if(!paused) {
    if(!read_frame()) {  next_file(); }
  }
  //glutPostRedisplay();
}

void rewind_one() {
  seq_ifs.clear();
  long off = sizeof(Frame); off *= -2;
  seq_ifs.seekg( off, ifstream::cur );
  if(!seq_ifs){
    seq_ifs.clear();
    seq_ifs.seekg(0, ios_base::beg);
  }
  if(!read_frame()){
    printf("failed to read file\n");
  }
}

//void key(unsigned char k, int x, int y) {
//  switch(k) {
//    case 'q':
//    case 27:
//      //glutLeaveMainLoop();
//      exit(0);
//      break;
//    case 'f':
//      if(!read_frame()) next_file();
//      break;
//    case 'b':
//      rewind_one();
//      break;
//
//    case 'p': paused = !paused; break;
//    case 'x': x_angle += .5; break;
//    case 'X': x_angle -= .5; break;
//    case 'y': y_angle += .5; break;
//    case 'Y': y_angle -= .5; break;
//  }
//}

//void display() {
//  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
//  glViewport(0,0,win_w(),win_h());
//  draw(frame);
//  glutSwapBuffers();
//}

//int main(int argc, char* argv[])
//{
//  // INIT
//  glutInit(&argc,argv);
//  glutInitDisplayMode(  GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
//  glutCreateWindow("TOF MOCAP PLAYER-Varun Ganapathi");
//
//  // IF YOU HAVE GLEW
//  // glewInit();
//
//  if(argc < 2){
//    printf("please specify directory containing Stanford TOF MOCAP data set");
//    return -1;
//  }
//  data_set_dir = argv[1];
//  printf("Key commands: \n");
//  printf("q : quit \n");
//  printf("p : pause \n");
//  printf("f : next frame\n");
//  printf("b : previous frame\n");
//  printf("xXyY: rotate about axis\n");
//
//  glutReshapeWindow(512, 512);
//  glutDisplayFunc(display);
//  //glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
//  glutReshapeFunc(reshape);
//  glutKeyboardFunc(key);
//  glutIdleFunc(idle);
//  glClearColor(1,1,1,1);
//  next_file();
//  glutMainLoop();
//  return 0;
//}

//float color_map[][3] =  {
//  0 ,       0 ,  0.5625,
//  0 ,       0 ,  0.6250,
//  0 ,       0 ,  0.6875,
//  0 ,       0 ,  0.7500,
//  0 ,       0 ,  0.8125,
//  0 ,       0 ,  0.8750,
//  0 ,       0 ,  0.9375,
//  0 ,       0 ,  1.0000,
//  0 ,  0.0625 ,  1.0000,
//  0 ,  0.1250 ,  1.0000,
//  0 ,  0.1875 ,  1.0000,
//  0 ,  0.2500 ,  1.0000,
//  0 ,  0.3125 ,  1.0000,
//  0 ,  0.3750 ,  1.0000,
//  0 ,  0.4375 ,  1.0000,
//  0 ,  0.5000 ,  1.0000,
//  0 ,  0.5625 ,  1.0000,
//  0 ,  0.6250 ,  1.0000,
//  0 ,  0.6875 ,  1.0000,
//  0 ,  0.7500 ,  1.0000,
//  0 ,  0.8125 ,  1.0000,
//  0 ,  0.8750 ,  1.0000,
//  0 ,  0.9375 ,  1.0000,
//  0 ,  1.0000 ,  1.0000,
//  0.0625 ,  1.0000 ,  0.9375,
//  0.1250 ,  1.0000 ,  0.8750,
//  0.1875 ,  1.0000 ,  0.8125,
//  0.2500 ,  1.0000 ,  0.7500,
//  0.3125 ,  1.0000 ,  0.6875,
//  0.3750 ,  1.0000 ,  0.6250,
//  0.4375 ,  1.0000 ,  0.5625,
//  0.5000 ,  1.0000 ,  0.5000,
//  0.5625 ,  1.0000 ,  0.4375,
//  0.6250 ,  1.0000 ,  0.3750,
//  0.6875 ,  1.0000 ,  0.3125,
//  0.7500 ,  1.0000 ,  0.2500,
//  0.8125 ,  1.0000 ,  0.1875,
//  0.8750 ,  1.0000 ,  0.1250,
//  0.9375 ,  1.0000 ,  0.0625,
//  1.0000 ,  1.0000 ,       0,
//  1.0000 ,  0.9375 ,       0,
//  1.0000 ,  0.8750 ,       0,
//  1.0000 ,  0.8125 ,       0,
//  1.0000 ,  0.7500 ,       0,
//  1.0000 ,  0.6875 ,       0,
//  1.0000 ,  0.6250 ,       0,
//  1.0000 ,  0.5625 ,       0,
//  1.0000 ,  0.5000 ,       0,
//  1.0000 ,  0.4375 ,       0,
//  1.0000 ,  0.3750 ,       0,
//  1.0000 ,  0.3125 ,       0,
//  1.0000 ,  0.2500 ,       0,
//  1.0000 ,  0.1875 ,       0,
//  1.0000 ,  0.1250 ,       0,
//  1.0000 ,  0.0625 ,       0,
//  1.0000 ,       0 ,       0,
//  0.9375 ,       0 ,       0,
//  0.8750 ,       0 ,       0,
//  0.8125 ,       0 ,       0,
//  0.7500 ,       0 ,       0,
//  0.6875 ,       0 ,       0,
//  0.6250 ,       0 ,       0,
//  0.5625 ,       0 ,       0,
//  0.5000 ,       0 ,       0
//};
//
//int color_map_size = sizeof(color_map)/( sizeof(color_map[0]));

}
