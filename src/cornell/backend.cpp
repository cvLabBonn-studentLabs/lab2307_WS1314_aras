/*
 * backend.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: neek
 */


#include "cornell/backend.h"
#include "readData.cpp"

namespace io {

DataIOCornell::DataIOCornell() {}


bool DataIOCornell::read_next_frame(cv::Mat& frame_rgb, cv::Mat& frame_depth) {
	std::string data_location = "data/cornell/Subject1_annotations/arranging_objects/";
	std::string filename = "0510175554";
	boost::shared_ptr<cad120::readData> input_data =
			boost::make_shared<cad120::readData>(data_location, filename);

    double **data;          //[JOINT_NUM][JOINT_DATA_NUM];
    int **data_CONF;        //[JOINT_NUM][JOINT_DATA_TYPE_NUM]
    double **pos_data;      //[POS_JOINT_NUM][POS_JOINT_DATA_NUM];
    int *pos_data_CONF;     //[POS_JOINT_NUM]

	data = new double*[cad120::JOINT_NUM];
	data_CONF = new int*[cad120::JOINT_NUM];
	for(int i = 0;i < cad120::JOINT_NUM; i++) {
	        data[i] = new double[cad120::JOINT_DATA_NUM];
	        data_CONF[i] = new int[cad120::JOINT_DATA_TYPE_NUM];
	}

    pos_data = new double*[cad120::POS_JOINT_NUM];
    pos_data_CONF = new int[cad120::POS_JOINT_NUM];
    for(int i = 0; i < cad120::POS_JOINT_NUM; i++) {
        pos_data[i] = new double[cad120::POS_JOINT_DATA_NUM];
    }

    /*int ***IMAGE;			// [X_RES][Y_RES]
    IMAGE = new int**[cad120::X_RES];
    for(int i = 0; i < cad120::X_RES; i++) {
        IMAGE[i] = new int*[cad120::Y_RES];
        for (int j = 0;j < cad120::Y_RES; j++) {
            IMAGE[i][j] = new int[cad120::RGBD_data];
        }
    }*/

	while(input_data->readNextFrame(data, pos_data, data_CONF, pos_data_CONF, frame_rgb, frame_depth) ) {
		//png_to_pointcloud(rgb_img, depth_img, cloud_rgb);

		int x = cad120::readData::xPixelFromCoords(data[0][9], data[0][10], data[0][11]);
		int y = cad120::readData::yPixelFromCoords(data[0][9], data[0][10], data[0][11]);

		int x1 = cad120::readData::xPixelFromCoords(data[1][9], data[1][10], data[1][11]);
		int y1 = cad120::readData::yPixelFromCoords(data[1][9], data[1][10], data[1][11]);

		int x2 = cad120::readData::xPixelFromCoords(pos_data[0][0], pos_data[0][1], pos_data[0][2]);
		int y2 = cad120::readData::yPixelFromCoords(pos_data[0][0], pos_data[0][1], pos_data[0][2]);

		int centre_x = min(x1, x) + abs(x1 - x)/2;
		int centre_y = min(y1, y) + abs(y1 - y)/2;

		std::cerr << "X/Y: " << x << "/" << y << std::endl;
		std::cerr << "X/Y: " << x1 << "/" << y1 << std::endl;
		std::cerr << "CONF: " << pos_data_CONF[0] << std::endl;


		cv::circle(frame_depth,
		         cv::Point(x2, y2),
		         5.0,
		         cv::Scalar( 255, 255, 255 ),
		         -1,
		         8);

		//cv::Mat roi(rgb_img, cv::Rect(centre_x, centre_y, 10, 10));
		cv::imshow("Patch", frame_depth);
		cv::waitKey(0);
	}

    return true;
}

}
