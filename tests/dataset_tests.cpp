/*
 * mesh_tests.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: neek
 */



#include <iostream>
#include <gtest/gtest.h>
#include "data_io_backend.h"

using namespace io;


TEST(DataSetBackendTest, PatchIntersectionTest)
{
	cv::Point p1, p2;
	float area_truth;
	float full_area = static_cast<float>(pose::kDescriptorSize*pose::kDescriptorSize);

	p1.x = 5;
	p1.y = 1;
	p2.x = 6;
	p2.y = 0;

	// The number of edges
	area_truth = static_cast<float>(pose::kDescriptorSize - 1)*(pose::kDescriptorSize - 1)
									/ full_area;
	EXPECT_EQ(DataIOBackend::intersection_ratio(p1, p2), area_truth);

	p1.x = 5;
	p1.y = 1;
	p2.x = pose::kDescriptorSize + 6;
	p2.y = pose::kDescriptorSize + 2;

	// The number of edges
	area_truth = 0.f;
	EXPECT_EQ(DataIOBackend::intersection_ratio(p1, p2), area_truth);

	p1.x = pose::kDescriptorSize + 6;
	p1.y = pose::kDescriptorSize + 2;
	p2.x = 7;
	p2.y = 3;

	// The number of edges
	area_truth = 1.f/full_area;
	EXPECT_EQ(DataIOBackend::intersection_ratio(p1, p2), area_truth);
}

