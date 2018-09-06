#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "common.h"

void find_sphere_and_mark(int cloudIndex, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud);

std::vector<uint32_t> colors{
  0x000000, 0xFFFF00, 0x1CE6FF, 0xFF34FF, 0xFF4A46, 0x008941, 0x006FA6, 0xA30059,
  0xFFDBE5, 0x7A4900, 0x0000A6, 0x63FFAC, 0xB79762, 0x004D43, 0x8FB0FF, 0x997D87,
  0x5A0007, 0x809693, 0xFEFFE6, 0x1B4400, 0x4FC601, 0x3B5DFF, 0x4A3B53, 0xFF2F80,
  0x61615A, 0xBA0900, 0x6B7900, 0x00C2A0, 0xFFAA92, 0xFF90C9, 0xB903AA, 0xD16100,
  0xDDEFFF, 0x000035, 0x7B4F4B, 0xA1C299, 0x300018, 0x0AA6D8, 0x013349, 0x00846F,
  0x372101, 0xFFB500, 0xC2FFED, 0xA079BF, 0xCC0744, 0xC0B9B2, 0xC2FF99, 0x001E09,
  0x00489C, 0x6F0062, 0x0CBD66, 0xEEC3FF, 0x456D75, 0xB77B68, 0x7A87A1, 0x788D66,
  0x885578, 0xFAD09F, 0xFF8A9A, 0xD157A0, 0xBEC459, 0x456648, 0x0086ED, 0x886F4C
                             };

int main(int argc, char** argv) {

   std::cout << "Loading plt file: " << "\n";

   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

   std::string file = "/home/controller/ceres/detect-sphere-build/img/PhoFrame(0007).ply";
   //std::string file = "/home/controller/ceres/detect-sphere-build/img/less5.ply";
   //std::string file = "/home/controller/ceres/detect-sphere-build/img/ball.ply";
   pcl::io::loadPLYFile(file, *cloud);

   std::cout << "Loaded plt file: " << file <<"\n"
             << "Width: " << cloud->width << "\n";
   pcl::PointCloud<pcl::PointXYZRGBNormal> &c = *cloud;

   showStats(*cloud);


   // Create the filtering object: downsample the dataset using a leaf size of 1cm
   pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
   vg.setInputCloud(cloud);
   vg.setLeafSize (1.f, 1.f, 1.f);
   vg.filter (*cloud_filtered);
   std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*


   pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
     tree->setInputCloud (cloud_filtered);
     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
     ec.setClusterTolerance (5); // 2cm
     ec.setMinClusterSize (100);
     ec.setMaxClusterSize (250000);
     ec.setSearchMethod(tree);
     ec.setInputCloud(cloud_filtered);
     ec.extract (cluster_indices);

     std::cout << "Found " << cluster_indices.size() << " clusters" << std::endl;
    
     int j = 0;
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
       pcl::ExtractIndices<pcl::PointXYZRGBNormal> eifilter(true); // Initializing with true will allow us to extract the removed indices
       pcl::PointIndices::Ptr indicesPtr (new pcl::PointIndices());
       indicesPtr->indices = it->indices;

       eifilter.setInputCloud(cloud_filtered);
       eifilter.setIndices(indicesPtr);
       eifilter.filter(*cluster);

       find_sphere_and_mark(j, cluster); 
       

       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
         uint32_t c = colors[j % (colors.size())];
         cloud_filtered->points[*pit].r = c >> 16 & 0x0000ff;
         cloud_filtered->points[*pit].g = c >> 8 & 0x0000ff;
         cloud_filtered->points[*pit].b = c & 0x0000ff;
       }

       j++;
     }


   pcl::io::savePLYFile("filtered_ply.ply", *cloud_filtered);

   std::cerr << "Saved " << cloud->points.size () << " data points to test_ply.ply." << std::endl;
}


void find_sphere_and_mark(int cloudIndex, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud)
{
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> segmentation;

   segmentation.setInputCloud(cloud);
   segmentation.setInputNormals(cloud);
   segmentation.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
   segmentation.setMethodType(pcl::SAC_RANSAC);
   segmentation.setDistanceThreshold(2);
   segmentation.setNormalDistanceWeight(0.1);
   segmentation.setOptimizeCoefficients(true);
   segmentation.setRadiusLimits(18,22);
   segmentation.setEpsAngle(1 / (180/3.141592654));
   segmentation.setMaxIterations(100000000);

   pcl::PointIndices inlierIndices;
   segmentation.segment(inlierIndices, *coefficients);

   if (inlierIndices.indices.size() == 0)
     cout << "Cluster: " << cloudIndex << "RANSAC nothing found" << "\n";
    else
    {
      cout << "Cluster: " << cloudIndex << "RANSAC found shape with [%d] points:" << inlierIndices.indices.size() << "\n";
      //for (int c=0; c<coefficients->values.size(); ++c)
      //    ROS_INFO("Coeff %d = [%f]", (int)c+1, (float)coefficients->values[c]);

      // mark the found inliers in green
      for (int m=0; m<inlierIndices.indices.size(); ++m)
      {
          cloud->points[inlierIndices.indices[m]].r = 255;
          cloud->points[inlierIndices.indices[m]].g = 0;
          cloud->points[inlierIndices.indices[m]].b = 0;
      }

      std::cout << "Cluster: " << cloudIndex << "Model coefficient: " << *coefficients << std::endl;
    }

}
