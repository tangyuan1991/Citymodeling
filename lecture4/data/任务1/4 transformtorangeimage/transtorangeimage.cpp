// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#pragma once
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <ctime>
#include <atomic>
#include <iostream>
#include <sstream>
#include <time.h>
#include <sys/timeb.h>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2\dnn.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/png_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/range_image_visualizer.h> 
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", pointCloud) == -1)
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}
	// Generate the data
	//for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
	//	for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
	//		pcl::PointXYZ point;
	//		point.x = 2.0f - y;
	//		point.y = y;
	//		point.z = z;
	//		pointCloud.points.push_back(point);
	//	}
	//}
	pointCloud.width = pointCloud.size();
	pointCloud.height = 1;

	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
	float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 1.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.001;
	float minRange = 0.0f;
	int borderSize = 2;

	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	//可视化深度图
	float* ranges = rangeImage.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage.width, rangeImage.height);
	pcl::io::saveRgbPNGFile("saveRangeImageRGB.png", rgb_image, rangeImage.width, rangeImage.height);

	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(rangeImage);
	
	std::cout << rangeImage << "\n";
}