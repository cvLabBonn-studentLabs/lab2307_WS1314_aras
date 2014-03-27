// Author:  Juergen Gall
// Contact: gall@vision.ee.ethz.ch
// Date: 18.05.2011
// Computer Vision Laboratory, ETH Zurich

// All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

// Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

// ##########################################################################

#include "NShow.h"

namespace NShow {

  // files 
  std::string image_file    = ""; 
  std::string image_path    = ""; 
  std::string camera_file   = "";
  std::string cameraRGB_file= "";
  std::string mesh_file     = "";
  std::string anno_file     = "";
  std::string tree_file     = "";
  std::string treeFace_file = "";
  std::string treeHand_file = "";
  std::string fileWrite_path = "";

  // paths
  std::string output_path = ""; 
  std::string est_path = ""; 

  // trees
  int offset = 0;
  int trees  = 0;
  int sample_images = 0;
  int sample_object = 0;

  // frame range
  int start_frame = 0;
  int end_frame = 0;
}


