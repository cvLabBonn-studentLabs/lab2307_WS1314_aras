/*
 * backend.h
 *
 *  Created on: Mar 18, 2014
 *      Author: neek
 */

#ifndef BACKEND_CORNELL_H_
#define BACKEND_CORNELL_H_

#include "data_io_backend.h"

namespace io {

class DataIOCornell : public DataIOBackend {
public:
	DataIOCornell();
	bool read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth);
};

}

#endif /* BACKEND_H_ */
