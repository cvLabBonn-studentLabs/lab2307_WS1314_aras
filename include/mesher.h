/*
 * mesher.h
 *
 *  Created on: Mar 6, 2014
 *      Author: neek
 */

#ifndef MESHER_H_
#define MESHER_H_

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

//REMOVE
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/filter.h>

#include "constants.h"

namespace mesher {

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

	typedef struct
	{
		float x;
		float y;
		float z;
		int count;
	} XYZPoint;

	typedef boost::property<boost::edge_weight_t, float> EdgeProperty;
	typedef boost::adjacency_list<boost::listS,
									boost::vecS,
									boost::undirectedS,
									boost::no_property,
									EdgeProperty> Graph;

	typedef boost::graph_traits < Graph >::vertex_descriptor vertex_descriptor;

	void process(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out);

	void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
						pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
						float cell_size);

	class Mesh {
	public:
		Mesh(float radius, int K);
		~Mesh();
		void compute_with_flow(const cv::Mat depth_img, const cv::Mat opt_flow, float z_scale);
		void set_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) { cloud_ = cloud; }
		void set_zfilter(float lower, float upper);
		void colour_mat(cv::Mat& image);
		void colour_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_pcl);
		void compute();
		void set_edge_threshold(float threshold) { edge_lenth_threshold_ = threshold; };
		Graph get_graph() { return graph_; };
		void get_interest_points(std::vector<cv::Point>& points,
										std::vector<cv::Point>& orientations);

		// DEBUG
		void mark_centroids(cv::Mat& image, float colour);

		int size() { return boost::num_vertices(graph_); };
	private:
		void expand(const int index,
					pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in,
					pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree,
					bool* expanded);

		void add_point(float x, float y, float z);

		int num_interest_points_;
		int orientation_delta_;
		float edge_lenth_threshold_;
		Graph graph_;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
		std::vector<int> centroids_;
		std::vector<int> index_map_;
		std::vector<int> interest_points_;
		std::vector<int> interest_points_o_;

		static const int kCenterMassNumNeighbours = 5;
		static const int kMinMeshSize = 150;
		static const float kSubtractBackgroundZ = 0.1f;
		static const int kSampleStep = 2;
	};

}


#endif /* MESHER_H_ */
