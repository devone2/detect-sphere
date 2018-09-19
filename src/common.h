#ifndef COMMON
#define COMMON

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

struct SegResult {
  pcl::ModelCoefficients::Ptr coef;
  pcl::PointIndices::Ptr ind;
  pcl::PointIndices::Ptr clusterInd;
  bool is_empty() {
    return ind == nullptr || ind->indices.size()==0;
  }

  int size()
  {
    return ind!= nullptr ? ind->indices.size() : 0;
  }
};

void showStats(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);

void colorPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,const pcl::PointIndices& indices, int colorIndex);
#endif // COMMON

void detect_plane(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,pcl::PointIndices& indices, pcl::ModelCoefficients::Ptr& coefficients);

bool remove_plane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, int max);

void find_sphere_and_mark(int cloudIndex, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, SegResult& result);

void extract_indices(std::vector<int>& selected, std::vector<int>& orig, std::vector<int>& result);

void find(std::string inputFile, std::string outFile, SegResult& result);
