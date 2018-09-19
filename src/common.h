#ifndef COMMON
#define COMMON

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

void showStats(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);

void colorPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,const pcl::PointIndices& indices, int colorIndex);
#endif // COMMON

void detect_plane(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,pcl::PointIndices& indices, pcl::ModelCoefficients::Ptr& coefficients);

bool remove_plane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, int max);
