/*
 * segmentation.h
 *
 *  Created on: Mar 5, 2014
 *      Author: neek
 */

#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/search/kdtree.h>

namespace segmentation {

	typedef pcl::PointXYZRGBNormal PointTypeFull;
	typedef pcl::PointXYZRGB PointTypeIO;

	void process(pcl::PointCloud<PointTypeIO>::ConstPtr cloud_in,
					pcl::PointCloud<PointTypeIO>::Ptr cloud_out);
}


#endif /* SEGMENTATION_H_ */
