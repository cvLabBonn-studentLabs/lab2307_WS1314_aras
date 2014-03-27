/*
 * Mesh.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: neek
 */


#include "mesher.h"


#define BENCHMARK 1
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
		edge_lenth_threshold_(radius),
		num_interest_points_(K),
		orientation_delta_(pose::kOrientationDelta) {}
Mesh::~Mesh() {}

void Mesh::set_zfilter(float lower, float upper) {
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (lower, upper);
    pass.filter (*cloud_);
}


void Mesh::compute_with_flow(const cv::Mat depth_img, const cv::Mat opt_flow, float z_scale) {
	assert(depth_img.type() == CV_32F && opt_flow.type() == CV_32F);

	cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	int point_idx = 0;
	const int cols = depth_img.cols;
	const int rows = depth_img.rows;
	int image_size = cols*rows;

#ifdef BENCHMARK
t.start();
#endif

	index_map_ = std::vector<int>(image_size, -1);
	for(int row = 0; row < rows; ++row) {
	    const float* p = depth_img.ptr<float>(row);
	    for(int col = 0; col < cols; ++col) {

	    	// adding only non-background points
	    	if (*p > kSubtractBackgroundZ) {
				add_point(col, row, *p*z_scale);
				index_map_[row*cols + col] = point_idx++;
			}

	    	p++;
	    }
	}

#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) INDEX_MAP + CLOUD: " << t.duration() << "ms" << std::endl;
#endif

#ifdef BENCHMARK
t.start();
#endif

	// Building the graph
	for(int row = 1; row < rows - 1; ++row) {
	    const float* p = depth_img.ptr<float>(row);
	    const float* p_flow = opt_flow.ptr<float>(row);

	    p++; // skipping 1st column
	    for(int col = 1; col < cols - 1; ++col) {

	    	// Computing the distance to the neighbours of p
	    	if (*p > kSubtractBackgroundZ) {

	    		// Searching the neighbourhood
	    		int p_idx = row*cols + col;
	    		int indices[] = {-cols,		// top
								-cols - 1,	// top left
								-cols + 1,	// top right
								-1,			// left
								1,			// right
								cols,		// bottom
								cols - 1,	// bottom left
								cols + 1	// bottom right
	    						};

	    		for (int k = 0; k < 8; k++) {
	    			const float* nb = p + indices[k];
	    			if (*nb < kSubtractBackgroundZ) continue;

	    			const float* nb_flow = p_flow + indices[k];
	    			float edge_weight = std::sqrt(2.f + (*p - *nb)*(*p - *nb)*z_scale*z_scale);
	    			float flow_cost = std::abs(*p_flow - *nb_flow);
	    			float cost = edge_weight + flow_cost*pose::kOpticalFlowThreshold;

	    			if (cost < edge_lenth_threshold_) {
	    				boost::add_edge(index_map_[p_idx], index_map_[p_idx + indices[k]], edge_weight, graph_);
	    			}
	    		}
	    	}

	    	p++;
	    	p_flow++;
	    }
	}

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

    while (component.size() != index_map_.size()) index_map_.pop_back();

    XYZPoint init_point;
    init_point.x = 0.0f;
    init_point.y = 0.0f;
    init_point.z = 0.0f;
    init_point.count = 0;
	std::vector<XYZPoint> centroids(num, init_point);
	centroids_ = std::vector<int>(num, -1);
    for (size_t i = 0; i < component.size(); ++i) {
		pcl::PointXYZ point_pcl = cloud_->at(i);
		XYZPoint* p = &centroids[component[i]];
		p->x += point_pcl.x;
		p->y += point_pcl.y;
		p->z += point_pcl.z;
		p->count++;
    }

    // Normalizing
    for (std::vector<XYZPoint>::iterator it = centroids.begin();
    		it != centroids.end(); ++it) {
    	it->x = it->x / it->count;
    	it->y = it->y / it->count;
    	it->z = it->z / it->count;
    }

	// Finding the closest point in the mesh
	std::vector<float> k_sqr_distances(num, infinity);

    for (size_t i = 0; i < component.size(); ++i) {
		pcl::PointXYZ point_pcl = cloud_->at(i);
		XYZPoint p = centroids[component[i]];
		if (p.count < pose::kConnectedComponentMinSize) continue;

		float delta = (p.x - point_pcl.x)*(p.x - point_pcl.x)
						+ (p.y - point_pcl.y)*(p.y - point_pcl.y)
						+ (p.z - point_pcl.z)*(p.z - point_pcl.z);
		if (delta < k_sqr_distances[component[i]]) {
			k_sqr_distances[component[i]] = delta;
			centroids_[component[i]] = i;
		}
    }

#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Finding centroids: " << t.duration() << "ms" << std::endl;
#endif

#ifdef BENCHMARK
t.start();
#endif

    int centroid_idx = -1;
    for (std::vector<XYZPoint>::iterator it = centroids.begin();
    		it != centroids.end(); ++it) {
		centroid_idx++;

		int k_index = centroids_[centroid_idx];
		if (k_index < 0) continue;

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
}

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
	std::vector<XYZPoint> centroids(num, init_point);
	centroids_ = std::vector<int>(num, -1);
    for (size_t i = 0; i < component.size(); ++i) {
		pcl::PointXYZ point_pcl = cloud_->at(i);
		XYZPoint *p = &centroids[component[i]];
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
    for (std::vector<XYZPoint>::iterator it = centroids.begin();
    		it != centroids.end(); ++it) {
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

		centroids_[centroid_idx] = k_index;

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
}

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

	for(int row = 0; row < image.rows; ++row) {
	    uchar* p = image.ptr<uchar>(row);
	    for(int col = 0; col < image.cols; ++col) {
	    	int idx = row*image.cols + col;
	    	if (idx >= index_map_.size())
	    		break;

	    	if (index_map_[idx] < 0) {
	    		p += 3;
	    		continue;
	    	}

	    	cv::Scalar colour = colours[component[index_map_[row*image.cols + col]]];
	    	*p = (uchar)colour[0]; p++;
	    	*p = (uchar)colour[1]; p++;
	    	*p = (uchar)colour[2]; p++;
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
		new_point.x = cloud_->at(i).x;
		new_point.y = cloud_->at(i).y;
		new_point.z = cloud_->at(i).z;
		new_point.rgb = colours[component[i]];

		if (centroids_[component[i]] == i) {
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
		new_point.x = cloud_->at(point_idx).x;
		new_point.y = cloud_->at(point_idx).y;
		new_point.z = cloud_->at(point_idx).z;
		new_point.rgb = color_global.float_value;

		cloud_pcl->push_back(new_point);
    }

    color_global.Red = 0;
    color_global.Blue = 0;
    color_global.Green = 0;
    for (size_t i = 0; i < interest_points_.size(); ++i) {
    	int point_idx = interest_points_o_[i];
    	pcl::PointXYZRGB new_point;
		new_point.x = cloud_->at(point_idx).x;
		new_point.y = cloud_->at(point_idx).y;
		new_point.z = cloud_->at(point_idx).z;
		new_point.rgb = color_global.float_value;

		cloud_pcl->push_back(new_point);
    }
}

void Mesh::expand(const int index,
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
												edge_lenth_threshold_,
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
}

void Mesh::add_point(float x, float y, float z) {
	pcl::PointXYZ new_point;
	new_point.x = x;
	new_point.y = y;
	new_point.z = z;
	cloud_->push_back(new_point);
}

void Mesh::mark_centroids(cv::Mat& image, float colour) {
	for (std::vector<int>::iterator it = centroids_.begin(); it != centroids_.end(); ++it) {
		if (*it < 0) continue; // CC is too small

		int x = cloud_->points[*it].x;
		int y = cloud_->points[*it].y;


		cv::circle(image, cv::Point(x, y), 1.0, colour, -1, 8);

		//std::cerr << "Centroid: [" << *it << "] X: " << x << " Y: " << y << " Z: " << cloud_->points[*it].z << std::endl;
	}
}

void Mesh::get_interest_points(std::vector<cv::Point>& points, std::vector<cv::Point>& orientations) {

	for (int i = 0; i < interest_points_.size(); i++) {
		cv::Point p;
		p.x = cloud_->points[interest_points_[i]].x;
		p.y = cloud_->points[interest_points_[i]].y;
		points.push_back(p);

//		std::cerr << "IP: [" << i << "] X: " << p.x <<
//					" Y: " << p.y <<
//					" Z: " << cloud_->points[interest_points_[i]].z << std::endl;

		cv::Point o;
		o.x = cloud_->points[interest_points_o_[i]].x;
		o.y = cloud_->points[interest_points_o_[i]].y;
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