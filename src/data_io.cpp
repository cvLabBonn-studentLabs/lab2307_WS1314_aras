#include "data_io.h"


typedef union
{
  struct
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

namespace io {

	DataIO::DataIO(Dataset dtype) : dtype_(dtype) {

		switch(dtype_)
		{
		case ETHZ:
		{
			backend_ = boost::make_shared<DataIOETHZ>();
		}
			break;
		case CornellCAD120:
		{
			backend_ = boost::make_shared<DataIOCornell>();
		}
			break;
		case StanfordEval:
		{
			backend_ = boost::make_shared<DataIOStanford>();
		}
			break;
		case Bonn:
		{
			backend_ = boost::make_shared<DataIOBonn>();
		}
			break;
		default:
			break;
		}

	}

	DataIO::~DataIO() {}

	float DataIO::getScaleZ() {
		return backend_->getScaleZ();
	}

//	void DataIO::read(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
//						pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_rgb) {
//
//		cv::Mat rgb_img, depth_img;
//		backend_->read_next_frame(rgb_img, depth_img);
//
//		if (depth_img.cols != 0) {
//			if (rgb_img.cols != 0) {
//				png_to_pointcloud(rgb_img, depth_img, cloud_rgb);
//			} else {
//				png_to_pointcloud(depth_img, cloud);
//			}
//		}
//	}

//	bool DataIO::read(cv::Mat& depth_img,
//						pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//
//		cv::Mat rgb_img_dummy;
//		if (backend_->read_next_frame(rgb_img_dummy, depth_img)) {
//			png_to_pointcloud(depth_img, cloud);
//			return true;
//		}
//
//		return false;
//	}

	bool DataIO::read(cv::Mat& depth_img) {

		cv::Mat rgb_img_dummy;
		if (backend_->read_next_frame(rgb_img_dummy, depth_img))
			return true;

		return false;
	}

	void DataIO::extract_data() {
		backend_->extract_parts();
	}

//	void DataIO::png_to_pointcloud(const cv::Mat& depth_img,
//									pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//		assert(depth_img.type() == CV_32F);
//
//		for (int row = 0; row < depth_img.rows; ++row) {
//		    const float* depth_p = depth_img.ptr<float>(row);
//
//		    for (int col = 0; col < depth_img.cols; ++col) {
//		        if (*depth_p != 0) {
//					pcl::PointXYZ point;
//
//					point.x = static_cast<float>(col);
//					point.y = static_cast<float>(row);
//					point.z = (*depth_p++)*ALPHA_DEPTH;
//
//					cloud->push_back(point);
//		        }
//		    }
//		}
//	}

	void DataIO::get_ground_truth(cv::Point& centre_head,
											cv::Point& centre_left_hand,
											cv::Point& centre_right_hand,
											cv::Point& centre_left_foot,
											cv::Point& centre_right_foot) {
		backend_->get_ground_truth(centre_head, centre_left_hand, centre_right_hand, centre_left_foot, centre_right_foot);
	}


//	void DataIO::png_to_pointcloud(const cv::Mat& rgb_img,
//									const cv::Mat& depth_img,
//									pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
//
//		// Fix the size mismatch between RGB and depth image
//		assert(rgb_img.size() == depth_img.size());
//		assert(rgb_img.type() == CV_8UC3);
//		assert(depth_img.type() == CV_8UC1);
//
//		double minVal, maxVal;
//		cv::minMaxLoc(depth_img, &minVal, &maxVal);
//
//		cloud->resize(cv::countNonZero(depth_img));
//
//		float rgb_2_depth_width = static_cast<float>(rgb_img.cols) / depth_img.cols;
//		float rgb_2_depth_height = static_cast<float>(rgb_img.rows) / depth_img.rows;
//
//		const float alpha_depth = 2.0;
//		for (int row = 0; row < depth_img.rows; ++row) {
//		    const uchar* depth_p = depth_img.ptr<uchar>(row);
//
//		    int row_rgb = static_cast<int>(round(row * rgb_2_depth_height));
//		    const uchar* rgb_p = rgb_img.ptr(row_rgb);
//
//		    for (int col = 0; col < depth_img.cols; ++col) {
//		        if (*depth_p != 0) {
//		        	const uchar* rgb_pp = rgb_p + 3*static_cast<int>(round(col * rgb_2_depth_width));
//
//		        	uchar depth = *depth_p;
//					uchar b = *rgb_pp++;
//					uchar g = *rgb_pp++;
//					uchar r = *rgb_pp++;
//
//					pcl::PointXYZRGB point;
//
//					point.x = static_cast<float>(col);
//					point.y = static_cast<float>(row);
//					point.z = depth*ALPHA_DEPTH;
//
//					// Fill in color
//					RGBValue color;
//					color.Red   = r;
//					color.Green = g;
//					color.Blue  = b;
//					color.Alpha = 0;
//					point.rgb = color.float_value;
//
//					cloud->push_back(point);
//		        }
//
//		        depth_p++;
//		    }
//		}
//
//	}

	bool DataIO::load_float_image(std::string filename, cv::Mat& output) {

		IplImage* imInt   = cvCreateImage( cvSize(176,144) , IPL_DEPTH_32F , 1);
		IplImage* img = 0;
		img = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
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
			cvGetRawData( imInt, (uchar**)&outdata, &stepOut);
			stepOut /= so;

			// copy float -> uchar
			for( int y = 0; y < img->height; y++, indata += stepIn, outdata += stepOut) {
				int m = 0;
				for( int k=0; k < so; k++)
					for( int l = k; l < imInt->width*so; l+=so ) {
						*((uchar*)(outdata)+l*si) = *((uchar*)(indata) + m*si);
						m++;
					}
			}

			cvReleaseImage(&img);

		} else {

			std::cout << "Could not find " << filename << std::endl;
		}

		output = cv::Mat(imInt);

		return res;
	}
}
