#ifndef COMMON
#define COMMON

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

void showStats(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);

void colorPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, pcl::PointIndices indices, int red);

#endif // COMMON

