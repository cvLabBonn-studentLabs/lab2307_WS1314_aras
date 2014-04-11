/*
 * mesher.h
 *
 *  Created on: Mar 6, 2014
 *      Author: neek
 */

#ifndef MESHER_H_
#define MESHER_H_

#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

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
		int index;
	} XYZPoint;

	typedef boost::property<boost::edge_weight_t, float> EdgeProperty;
	typedef boost::adjacency_list<boost::listS,
									boost::vecS,
									boost::undirectedS,
									boost::no_property,
									EdgeProperty> Graph;

	typedef boost::graph_traits < Graph >::vertex_descriptor vertex_descriptor;

	class Mesh {
	public:
		Mesh(float radius, int K);
		~Mesh();

		void compute(const cv::Mat depth_img, float z_scale);
		void compute(const cv::Mat depth_img, float z_scale, const cv::Mat opt_flow);
		void colour_mat(cv::Mat& image);

		void set_edge_threshold(float threshold) { edge_length_threshold_ = threshold; };
		Graph get_graph() { return graph_; };
		void get_interest_points(std::vector<cv::Point>& points,
										std::vector<cv::Point>& orientations);

		// For DEBUG purposes
		void mark_centroids(cv::Mat& image, float colour);
		void set_cost_function(float(*cost_function)(const float, const float)) { cost_function_ = cost_function; };
		int size() { return boost::num_vertices(graph_); };
	private:
		void add_point(float x, float y, float z, int index);
		void compute_(const cv::Mat opt_flow);
		void build_mesh_();
		static float default_cost_function(float edge_weight, float flow_cost) { return edge_weight + flow_cost*pose::kOpticalFlowThreshold; };
		float l22(XYZPoint& p1, XYZPoint& p2) {
			float norm = static_cast<float>(kSampleStep)*static_cast<float>(kSampleStep);
			return ((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z))/norm; };

		int num_interest_points_;
		int orientation_delta_;
		float edge_length_threshold_;
		Graph graph_;
		std::vector<XYZPoint> cloud_points_;
		std::vector<XYZPoint> centroids_;
		std::vector<int> index_map_;
		std::vector<int> interest_points_;
		std::vector<int> interest_points_o_;
		std::vector<int> components_;
		int num_of_cc_;

		float (*cost_function_)(const float p1, const float p2);

		static const int kCenterMassNumNeighbours = 5;
		static const int kMinMeshSize = 5;
		static const float kSubtractBackgroundZ = 0.f;
		static const int kSampleStep = 2;
	};

}


#endif /* MESHER_H_ */
