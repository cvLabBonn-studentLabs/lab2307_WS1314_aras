#ifndef DATA_IO_H_
#define DATA_IO_H_


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <pcl/point_types.h>
//#include <pcl/common/projection_matrix.h>
#include <boost/make_shared.hpp>
#include "data_io_backend.h"
#include "ethz/backend.h"
#include "stanford/backend.h"
#include "cornell/backend.h"
#include "bonn/backend.h"

namespace io {

class DataIO {
public:
	DataIO(Dataset d);
	~DataIO();

//	void read(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_rgb);
//
//	bool read(cv::Mat& depth_image,
//				pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

	bool read(cv::Mat& depth_image);

	void get_ground_truth(cv::Point& centre_head,
									cv::Point& centre_left_hand,
									cv::Point& centre_right_hand,
									cv::Point& centre_left_foot,
									cv::Point& centre_right_foot);

	void extract_data();

	float getScaleZ();
private:

//	void png_to_pointcloud(const cv::Mat& rgb_img,
//							const cv::Mat& depth_img,
//							pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

//	void png_to_pointcloud(const cv::Mat& depth_img,
//							pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

	Dataset dtype_;
	boost::shared_ptr<DataIOBackend> backend_;
	static const float ALPHA_DEPTH = 100.0;

protected:
	bool load_float_image(std::string filename, cv::Mat& out);

};

}

#endif /* DATA_IO_H_ */
