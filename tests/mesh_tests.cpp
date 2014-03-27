/*
 * mesh_tests.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: neek
 */



#include <iostream>
#include <gtest/gtest.h>
#include "mesher.h"

using namespace mesher;


TEST(MeshTest, CreatesCorrectMesh)
{
	const size_t num_cc = 3;
	const size_t num_edges = 29;
	const size_t num_vertices = 16;
	const float points[] = { 1.0f, 7.0f, 2.0f,	// #1 CC
								1.0f, 8.0f, 2.0f,
								2.0f, 8.0f, 2.0f,

								5.0f, 6.0f, 1.0f,	// #2 CC
								5.0f, 7.0f, 1.0f,
								6.0f, 6.0f, 1.0f,
								6.0f, 7.0f, 1.0f,

								2.0f, 3.0f, 0.0f,	// #3 CC
								2.0f, 2.0f, 0.0f,
								2.0f, 1.0f, 0.0f,
								3.0f, 3.0f, 0.0f,
								3.0f, 2.0f, 0.0f,
								3.0f, 1.0f, 0.0f,
								4.0f, 3.0f, 0.0f,
								4.0f, 2.0f, 0.0f,
								4.0f, 1.0f, 0.0f
	};

	// Creating a point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize (num_vertices);

	for (size_t i = 0; i < num_vertices; i++) {
		cloud->points[i].x = points[3*i];
		cloud->points[i].y = points[3*i + 1];
		cloud->points[i].z = points[3*i + 2];
	}

	Mesh m(1.42f, 10);
	m.set_pointcloud(cloud);
	m.compute();

	// The number of connected components
    std::vector<int> component(m.size());
    int num = boost::connected_components(m.get_graph(), &component[0]);
	EXPECT_EQ(num_cc, num);

	// The number of edges
	int num_edges_ = 0;
	Graph g = m.get_graph();

	boost::graph_traits<Graph>::vertex_iterator vi, vi_end, next;
	boost::graph_traits<Graph>::adjacency_iterator ai, a_end, a_next;
	boost::tie(vi, vi_end) = boost::vertices(g);
	for (next = vi; next != vi_end; ++next) {
		boost::tie(ai, a_end) = boost::adjacent_vertices(*next, g);
		for (a_next = ai; a_next != a_end; ++a_next) {
			num_edges_ += 1;
		}
	}

	// The number of edges
	EXPECT_EQ(num_edges, num_edges_/2);

	// The number of vertices
	EXPECT_EQ(num_vertices, m.size());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

