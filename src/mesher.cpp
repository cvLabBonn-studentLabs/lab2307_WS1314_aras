/*
 * Mesh.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: neek
 */


#include "mesher.h"


//#define BENCHMARK 1
#define DEBUG 1

#ifdef BENCHMARK
#include "timer.h"
static timer::Timer t;
#endif

namespace mesher {

inline bool consider_edge(float z1, float z2, float z_scale, Graph graph_) {
	if (sqrt(1 + 1 + (z1-z2)*(z1-z2)*z_scale) < pose::kMeshDistanceThreshold) {

		return true;
	}

	return false;
}

const int infinity = std::numeric_limits<int>::max();

RGBValue color_global;
float get_random_colour() {

	color_global.Red   = rand() % 255;
	color_global.Green = rand() % 255;
	color_global.Blue  = rand() % 255;
	color_global.Alpha = 0;

	return color_global.float_value;
}


Mesh::Mesh(float radius, int K) :
		edge_length_threshold_(radius),
		num_interest_points_(K),
		orientation_delta_(pose::kOrientationDelta),
		cost_function_(NULL) {}

Mesh::~Mesh() {}

void Mesh::compute(const cv::Mat depth_img, float z_scale, const cv::Mat opt_flow) {
	// creating the point cloud
	const int cols = depth_img.cols;
	const int rows = depth_img.rows;
	const int image_size = cols*rows;

	#ifdef BENCHMARK
	t.start();
	#endif

	index_map_ = std::vector<int>(image_size, -1);
	for(int row = 0; row < rows; row += kSampleStep) {
		const float* p = depth_img.ptr<float>(row);
		for(int col = 0; col < cols; col += kSampleStep) {
			if (*p == 0.0f) continue;	// skipping the pixels with no depth information

			int index = row*cols + col;
			add_point(col, row, *p*z_scale, index);
			index_map_[index] = cloud_points_.size() - 1;
			p += kSampleStep;
		}
	}

	#ifdef BENCHMARK
	t.stop();
	std::cout << "(BENCHMARKING) INDEX_MAP + CLOUD (" << cloud_points_.size() << " vertices): " << t.duration() << "ms" << std::endl;
	#endif

	compute_(opt_flow);
}

void Mesh::compute(const cv::Mat depth_img, float z_scale) {
	cv::Mat image_of(depth_img.size(), CV_32F, 0.f);
	compute(depth_img, z_scale, image_of);
}


void Mesh::compute_(const cv::Mat opt_flow) {
	if (cost_function_ == NULL) {
		cost_function_ = &(Mesh::default_cost_function);
	}

	// creating the point cloud
	const int cols = opt_flow.cols;
	const int rows = opt_flow.rows;
	const int img_size = cols*rows;

	assert(opt_flow.type() == CV_32F);

#ifdef BENCHMARK
t.start();
#endif

	const int indices[] = {-cols,		// top
							-cols - 1,	// top left
							-cols + 1,	// top right
							-1,			// left
							1,			// right
							cols,		// bottom
							cols - 1,	// bottom left
							cols + 1	// bottom right
							};
	// To remove
	float min_val, max_val;


	// Building the graph
	for (std::vector<XYZPoint>::iterator it = cloud_points_.begin(); it != cloud_points_.end(); it++) {

		int row = it->index / cols;
		int col = it->index % cols;

		XYZPoint p = *it;
		const float* p_flow = opt_flow.ptr<float>(row) + col;

		// Searching the neighbourhood
		int p_idx = it->index;
		for (int k = 0; k < 8; k++) {
			int nb_idx = p_idx + kSampleStep*indices[k];
			if (nb_idx < 0 || nb_idx >= img_size || index_map_[nb_idx] == -1) continue; // skipping the neighbours that are out of scope or have no depth information

			XYZPoint nb = cloud_points_[index_map_[nb_idx]];
			const float* nb_flow = p_flow + kSampleStep*indices[k];
			float l22_distance = l22(p, nb);
			float flow_cost = std::abs(*p_flow - *nb_flow);
			float cost = cost_function_(l22_distance, flow_cost);

			min_val = std::min(l22_distance, min_val);
			max_val = std::max(l22_distance, max_val);

			if (cost < edge_length_threshold_) {
				assert(index_map_[nb_idx] >= 0 && index_map_[p_idx] >= 0 && index_map_[p_idx] < cloud_points_.size());
				boost::add_edge(index_map_[p_idx], index_map_[nb_idx], l22_distance, graph_);
			}
		}
	}

	build_mesh_();
}

void Mesh::build_mesh_() {


#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Building Mesh: " << t.duration() << "ms" << std::endl;
#endif

#ifdef BENCHMARK
t.start();
#endif

	// computing centroids and consequently the interest points
	interest_points_ = std::vector<int>();
	interest_points_o_ = std::vector<int>();
    std::vector<int> component(boost::num_vertices(graph_));
    int num = boost::connected_components(graph_, &component[0]);

    //while (component.size() != index_map_.size()) index_map_.pop_back();

    XYZPoint init_point;
    init_point.x = 0.0f;
    init_point.y = 0.0f;
    init_point.z = 0.0f;
    init_point.count = 0;
    init_point.index = -1;
	centroids_ = std::vector<XYZPoint>(num, init_point);
    for (size_t i = 0; i < component.size(); ++i) {
		XYZPoint cloud_point = cloud_points_[i];
		XYZPoint* p = &centroids_[component[i]];
		p->x += cloud_point.x;
		p->y += cloud_point.y;
		p->z += cloud_point.z;
		p->count++;
    }

    // Normalizing
    for (std::vector<XYZPoint>::iterator it = centroids_.begin();
    		it != centroids_.end(); ++it) {
    	it->x = it->x / it->count;
    	it->y = it->y / it->count;
    	it->z = it->z / it->count;
    }

	// Finding the closest point in the mesh
	std::vector<float> k_sqr_distances(num, infinity);

    for (size_t i = 0; i < component.size(); ++i) {
    	XYZPoint cloud_point = cloud_points_[i];
		XYZPoint p = centroids_[component[i]];

		float delta = (p.x - cloud_point.x)*(p.x - cloud_point.x)
								+ (p.y - cloud_point.y)*(p.y - cloud_point.y)
								+ (p.z - cloud_point.z)*(p.z - cloud_point.z);
		if (delta < k_sqr_distances[component[i]]) {
			k_sqr_distances[component[i]] = delta;
			centroids_[component[i]].index = i;
		}
    }

#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Finding centroids: " << t.duration() << "ms" << std::endl;
#endif


#ifdef BENCHMARK
t.start();
#endif

    for (std::vector<XYZPoint>::iterator it = centroids_.begin();
    		it != centroids_.end(); ++it) {

    	if (it->count < kMinMeshSize) continue;

		std::vector<vertex_descriptor> p(boost::num_vertices(graph_));
		std::vector<int> d(boost::num_vertices(graph_));
		vertex_descriptor s = boost::vertex(it->index, graph_);

		int K = static_cast<int>(round(num_interest_points_*static_cast<float>(it->count)/static_cast<float>(boost::num_vertices(graph_))));
		for (int k = 0; k < K; k++) {
			boost::dijkstra_shortest_paths(graph_, s,
					boost::predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, graph_))).
					distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, graph_))));

			boost::graph_traits <Graph>::vertex_iterator vi, vend;
			int max_weight = -1;
			int max_weight_vertex = -1;

			for (boost::tie(vi, vend) = boost::vertices(graph_); vi != vend; ++vi) {
				if (max_weight < d[*vi] && d[*vi] != infinity) {
					max_weight = d[*vi];
					max_weight_vertex = *vi;
				}
			}

			assert(component[max_weight_vertex] == component[it->index]);
			// adding a zero-weight edge
			boost::add_edge(it->index, max_weight_vertex, 0, graph_);
			interest_points_.push_back(max_weight_vertex);

			// Find the orientation (tracing the path back)
			int parent = p[max_weight_vertex];
			for (int j = 1; p[parent] && j < orientation_delta_; j++) {
				parent = p[parent];
			}

			interest_points_o_.push_back(parent);
		}
    }

#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Finding IP: " << t.duration() << "ms" << std::endl;
#endif
}

/*
void Mesh::compute() {

#ifdef BENCHMARK
t.start();
#endif

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_);

	bool expanded[cloud_->size()];
	for (size_t j = 0; j < cloud_->size(); ++j) expanded[j] = false;

	for (size_t i = 0; i < cloud_->size(); ++i) {
		if (!expanded[i]) {
			expand(i, cloud_, &kdtree, expanded);
		}
	}

#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Graph construction: " << t.duration() << "ms" << std::endl;
#endif

#ifdef BENCHMARK
t.start();
#endif

	// computing centroids and consequently the interest points
	interest_points_ = std::vector<int>();
	interest_points_o_ = std::vector<int>();
    std::vector<int> component(boost::num_vertices(graph_));
    int num = boost::connected_components(graph_, &component[0]);

    XYZPoint init_point;
    init_point.x = 0.0f;
    init_point.y = 0.0f;
    init_point.z = 0.0f;
    init_point.count = 0;
    init_point.index = -1;
	//std::vector<XYZPoint> centroids(num, init_point);
	centroids_ = std::vector<XYZPoint>(num, init_point);
    for (size_t i = 0; i < component.size(); ++i) {
		pcl::PointXYZ point_pcl = cloud_->at(i);
		XYZPoint *p = &centroids_[component[i]];
		p->x += point_pcl.x;
		p->y += point_pcl.y;
		p->z += point_pcl.z;
		p->count++;
    }

#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Finding the mean: " << t.duration() << "ms" << std::endl;
#endif

#ifdef BENCHMARK
t.start();
#endif

    // Normalizing
    int centroid_idx = -1;
    for (std::vector<XYZPoint>::iterator it = centroids_.begin();
    		it != centroids_.end(); ++it) {
		centroid_idx++;

    	float count = static_cast<float>(it->count);
		float mean_x = it->x / count;
		float mean_y = it->y / count;
		float mean_z = it->z / count;

		if (it->count < kMinMeshSize) continue;

		pcl::PointXYZ point_pcl(mean_x, mean_y, mean_z);

		// Finding the closest point in the mesh
		std::vector<int> k_indices;
		std::vector<float> k_sqr_distances;
		int num_neighbours =  kdtree.nearestKSearch(point_pcl,
												kCenterMassNumNeighbours,
												k_indices,
												k_sqr_distances);

		// Index of the center of mass in the original point cloud
		int k_index = -1;
		for (int i = 0; i < kCenterMassNumNeighbours; ++i) {
			if (component[k_indices[i]] == centroid_idx) {
				k_index = k_indices[i];
				break;
			}
		}

		if (k_index == -1) {
			std::cerr << "No center of mass for a mesh found (size " << it->count << ")" << std::endl;
			continue;
		}

		centroids_[centroid_idx].index = k_index;

		std::vector<vertex_descriptor> p(boost::num_vertices(graph_));
		std::vector<int> d(boost::num_vertices(graph_));
		vertex_descriptor s = boost::vertex(k_index, graph_);

		int K = static_cast<int>(round(num_interest_points_*static_cast<float>(it->count)/static_cast<float>(boost::num_vertices(graph_))));
		for (int k = 0; k < K; k++) {
			boost::dijkstra_shortest_paths(graph_, s,
					boost::predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, graph_))).
					distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, graph_))));

			boost::graph_traits <Graph>::vertex_iterator vi, vend;
			int max_weight = -1;
			int max_weight_vertex = -1;
			for (boost::tie(vi, vend) = vertices(graph_); vi != vend; ++vi) {
				if (max_weight < d[*vi] && d[*vi] != infinity) {
					max_weight = d[*vi];
					max_weight_vertex = *vi;
				}
				//std::cout << "distance(" << name[*vi] << ") = " << d[*vi] << ", ";
				//std::cout << "parent(" << name[*vi] << ") = " << name[p[*vi]] << std::endl;
			}

			// adding a zero-weight edge
			boost::add_edge(k_index, max_weight_vertex, 0, graph_);
			interest_points_.push_back(max_weight_vertex);

			// Find the orientation (tracing the path back)
			int parent = p[max_weight_vertex];
			for (int j = 1; p[parent] && j < orientation_delta_; j++) {
				parent = p[parent];
			}

			interest_points_o_.push_back(parent);
		}
    }

#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Finding IP: " << t.duration() << "ms" << std::endl;
#endif
}*/

void Mesh::colour_mat(cv::Mat& image) {
	assert(image.type() == CV_8UC3);

	// The number of connected components
    std::vector<int> component(boost::num_vertices(graph_));
    int num = boost::connected_components(graph_, &component[0]);
    std::vector<cv::Scalar> colours;
    srand (time(NULL));
    for (size_t i = 0; i < num; i++) {
    	colours.push_back(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
    }

	for(int row = 0; row < image.rows; row += kSampleStep) {
	    uchar* p = image.ptr<uchar>(row);
	    for(int col = 0; col < image.cols; col += kSampleStep) {
	    	int idx = row*image.cols + col;

	    	if (index_map_[idx] >= 0 && index_map_[idx] < component.size()) {

	    		if (centroids_[component[index_map_[idx]]].index == index_map_[idx]) {
					*p = (uchar)255; p++;
					*p = (uchar)255; p++;
					*p = (uchar)255; p++;
	    		} else {
					cv::Scalar colour = colours[component[index_map_[idx]]];
					*p = (uchar)colour[0]; p++;
					*p = (uchar)colour[1]; p++;
					*p = (uchar)colour[2]; p++;
	    		}
	    	} else {
				*p = (uchar)128; p++;
				*p = (uchar)128; p++;
				*p = (uchar)128; p++;
	    	}

	    	p += 3*(kSampleStep - 1);
	    }
	}
}

void Mesh::colour_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_pcl) {

	// The number of connected components
    std::vector<int> component(boost::num_vertices(graph_));
    int num = boost::connected_components(graph_, &component[0]);
    std::vector<float> colours;
    srand (time(NULL));
    for (size_t i = 0; i < num; i++) {
    	colours.push_back(get_random_colour());
    }

    color_global.Red = 255;
    color_global.Blue = 255;
    color_global.Green = 255;
    for (size_t i = 0; i < component.size(); ++i) {
		pcl::PointXYZRGB new_point;
		new_point.x = cloud_points_[i].x;
		new_point.y = cloud_points_[i].y;
		new_point.z = cloud_points_[i].z;
		new_point.rgb = colours[component[i]];

		if (centroids_[component[i]].index == i) {
			new_point.rgb = color_global.float_value;
		}

		cloud_pcl->push_back(new_point);
    }

    color_global.Red = 0;
    color_global.Blue = 0;
    color_global.Green = 255;
    for (size_t i = 0; i < interest_points_.size(); ++i) {
    	int point_idx = interest_points_[i];
    	pcl::PointXYZRGB new_point;
		new_point.x = cloud_points_[point_idx].x;
		new_point.y = cloud_points_[point_idx].y;
		new_point.z = cloud_points_[point_idx].z;
		new_point.rgb = color_global.float_value;

		cloud_pcl->push_back(new_point);
    }

    color_global.Red = 0;
    color_global.Blue = 0;
    color_global.Green = 0;
    for (size_t i = 0; i < interest_points_.size(); ++i) {
    	int point_idx = interest_points_o_[i];
    	pcl::PointXYZRGB new_point;
		new_point.x = cloud_points_[point_idx].x;
		new_point.y = cloud_points_[point_idx].y;
		new_point.z = cloud_points_[point_idx].z;
		new_point.rgb = color_global.float_value;

		cloud_pcl->push_back(new_point);
    }
}

/*void Mesh::expand(const int index,
					pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in,
					pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree,
					bool* expanded) {

	std::vector<int> point_idx;
	std::vector<float> square_distance;
	std::queue<int> bfs_queue;

	bfs_queue.push(index);
	while (bfs_queue.size() > 0) {
		int idx = bfs_queue.front();
		bfs_queue.pop();

		if (expanded[idx]) continue;

		// expanding the vertex
		int num_neighbours =  kdtree->radiusSearch(cloud_in->at(idx),
												edge_length_threshold_,
												point_idx,
												square_distance);

		if (num_neighbours > 1) {
			for (size_t j = 0; j < point_idx.size(); ++j) {
				if (!expanded[point_idx[j]] && idx != point_idx[j]) {

					// adding a new edge
					boost::add_edge(idx, point_idx[j], square_distance[j], graph_);

					// pushing the index to the queue for BFS
					bfs_queue.push(point_idx[j]);
				}
			}
		}

		expanded[idx] = true;
	}
}*/

void Mesh::add_point(float x, float y, float z, int index) {
	XYZPoint new_point;
	new_point.x = x;
	new_point.y = y;
	new_point.z = z;
	new_point.index = index;
	cloud_points_.push_back(new_point);
}

void Mesh::mark_centroids(cv::Mat& image, float colour) {
	for (std::vector<XYZPoint>::iterator it = centroids_.begin(); it != centroids_.end(); ++it) {
		if (it->count < kMinMeshSize) continue;

		int x = cloud_points_[it->index].x;
		int y = cloud_points_[it->index].y;


		cv::circle(image, cv::Point(x, y), 3.0, cv::Scalar(255, 255, 255), -1, 8);

		//std::cerr << "Centroid: [" << *it << "] X: " << x << " Y: " << y << " Z: " << cloud_->points[*it].z << std::endl;
	}
}

void Mesh::get_interest_points(std::vector<cv::Point>& points, std::vector<cv::Point>& orientations) {

	for (int i = 0; i < interest_points_.size(); i++) {
		cv::Point p;
		p.x = cloud_points_[interest_points_[i]].x;
		p.y = cloud_points_[interest_points_[i]].y;
		points.push_back(p);

//		std::cerr << "IP: [" << i << "] X: " << p.x <<
//					" Y: " << p.y <<
//					" Z: " << cloud_->points[interest_points_[i]].z << std::endl;

		cv::Point o;
		o.x = cloud_points_[interest_points_o_[i]].x;
		o.y = cloud_points_[interest_points_o_[i]].y;
		orientations.push_back(o);
	}
}

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
					pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
					float cell_size) {
	  // Create the filtering object
	  pcl::VoxelGrid<pcl::PointXYZ> sor;
	  sor.setInputCloud(cloud_in);
	  sor.setLeafSize(cell_size, cell_size, cell_size);
	  sor.filter(*cloud_out);
}

/*void process(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	downsample(cloud_in, cloud_filtered);

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud_filtered, "God knows what");
	while (!viewer.wasStopped()) {}

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);
	n.setInputCloud (cloud_filtered);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	std::cout << "Num. parts: " << parts.size() << std::endl;
	std::cout << "Num. states: " << states.size() << std::endl;
	std::cout << "Num. polygons: " << triangles.polygons.size() << std::endl;
	std::cout << "Num. cloud points: " << triangles.cloud.data.size() << std::endl;
	std::cout << "Size. cloud norm. points (before): " << cloud_with_normals->size() << std::endl;

	int i = 0;
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud_with_normals->begin();
			it != cloud_with_normals->end(); it++) {

		pcl::PointNormal p = *it;
		if (p.x != 0.0 && p.y != 0.0 && p.z != 0.0) {
			std::cout << "Point[" << i << "]: " << p << std::endl;
			std::cin.get();
		}

		i++;
	}*/

	//std::vector<int> sss;
	//pcl::removeNaNFromPointCloud(*cloud_with_normals, *cloud_with_normals, sss);
	//std::cout << "Size. cloud norm. points (after): " << cloud_with_normals->size() << std::endl;

	/*
	pcl::PLYWriter pw;
	pw.writeBinary("mesh.ply", triangles.cloud);

	cloud_out->resize(cloud_in->size());
	for (std::vector<pcl::Vertices>::iterator it = triangles.polygons.begin();
			it != triangles.polygons.end(); it++) {


		RGBValue rgb = get_random_colour();

	    pcl::PointXYZRGB new_p;
	    for (int j = 0; j < it->vertices.size(); ++j) {
	    	pcl::PointNormal p = cloud_with_normals->at(it->vertices[j]);
	        new_p.x = p.x;
	        new_p.y = p.y;
	        new_p.z = p.z;
	        new_p.rgb = rgb.float_value;

	        std::cout << "Index: " << it->vertices[j] << std::endl;
	        std::cout << "Normal: " << p << std::endl;
	        std::cout << "New point: " << new_p << std::endl;
	        std::cin.get();
	        cloud_out->push_back(new_p);
	    }
	}

	std::cout << "Size of the new cloud: " << cloud_out->size() << std::endl;*/

	//pcl::io::saveOBJFile ();
//}

}
