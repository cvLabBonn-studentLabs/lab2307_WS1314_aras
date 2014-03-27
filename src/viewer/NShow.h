// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

#ifndef Show_H
#define Show_H

#include <string>

namespace NShow {

  // files 
  extern std::string image_file; 
  extern std::string image_path; 
  extern std::string camera_file;
  extern std::string cameraRGB_file;
  extern std::string mesh_file;
  extern std::string anno_file;
  extern std::string tree_file;
  extern std::string treeFace_file;
  extern std::string treeHand_file;

  extern std::string fileWrite_path;

  // paths
  extern std::string output_path; 
  extern std::string est_path; 

  // trees
  extern int offset;
  extern int trees;
  extern int sample_images;
  extern int sample_object;

  // frame range
  extern int start_frame;
  extern int end_frame;
}

#endif

