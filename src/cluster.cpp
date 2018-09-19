#include <iostream>
#include <fstream>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include "common.h"

namespace fs = boost::filesystem;

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

void find_sphere_and_mark(int cloudIndex, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, SegResult& result);

void extract_indices(std::vector<int>& selected, std::vector<int>& orig, std::vector<int>& result);

void find(std::string inputFile, std::string outFile, SegResult& result);

int main(int argc, char** argv) {

  if(argc != 3) {
    std::cout << "Provide input file as an only argument";
    return 1;
  }

  std::string dir = argv[1];
  fs::path out_dir = fs::path(argv[2]);
  fs::path out_results = fs::path(out_dir);
    out_results /= "results.csv";
  fs::directory_iterator end_iter;

  std::ofstream result_file;
  result_file.open(out_results.string());
  result_file << "file;found;points;x;y;z;r\n";

  for ( fs::directory_iterator dir_itr( dir );
        dir_itr != end_iter;
        ++dir_itr )
    {
      if ( fs::is_regular_file( dir_itr->status() ))
            {
              fs::path infilename = dir_itr->path().filename();
              fs::path outfile = fs::path(out_dir) /= infilename;
              std::cout << dir_itr->path().filename() << " to " << outfile << "\n";
              SegResult result;
              find(dir_itr->path().string(),outfile.string(), result);
              result_file << infilename << ";" ;
                if(result.is_empty()) {
                  result_file << "N" << std::endl;
                } else {
                  auto c = result.coef->values;
                  result_file << "Y;" <<  result.size() << ";" << c[0] << ";" << c[1] << ";" << c[2] << ";" << c[3] << std::endl;
                }

            }
    }

   result_file.close();
}


void find_sphere_and_mark(int cloudIndex, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, SegResult& result)
{
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> segmentation;

   segmentation.setInputCloud(cloud);
   segmentation.setInputNormals(cloud);
   segmentation.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
   segmentation.setMethodType(pcl::SAC_RANSAC);
   segmentation.setDistanceThreshold(1.25);
   segmentation.setNormalDistanceWeight(0.1);
   segmentation.setOptimizeCoefficients(true);
   segmentation.setRadiusLimits(20,21);
   segmentation.setEpsAngle(1 / (180/3.141592654));
   segmentation.setMaxIterations(1000000);

   pcl::PointIndices::Ptr indx(new pcl::PointIndices());
   segmentation.segment(*indx, *coefficients);

   if (indx->indices.size() == 0)
     std::cout << "Cluster: " << cloudIndex << ". RANSAC nothing found" << "\n";
    else
    {
      std::cout << "Cluster: " << cloudIndex << ". RANSAC found shape with [%d] points:" << indx->indices.size() << "\n";
      double part = (double)indx->indices.size() / (double) cloud->points.size() * 100;
      std::cout << "Cluster: " << cloudIndex << ". Model coefficient: " << *coefficients << std::endl << "Points: " << indx->indices.size() << " ("<< part <<"%)" << std::endl;
    }

   result.coef = coefficients;
   result.ind = indx;

}

void extract_indices(std::vector<int>& selected, std::vector<int>& orig, std::vector<int>& result)
{
  for(int i=0;i<selected.size();i++) {
    result.push_back(orig[selected[i]]);
  }
}


void find(std::string inputFile, std::string outFile, SegResult& result)
{
  std::cout << "Loading plt file: "<< inputFile << "\n";

   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered0 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

   std::string file = inputFile;
   //   std::string file = "/home/controller/ceres/detect-sphere-build/img/PhoFrame(0007).ply";
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
   vg.setLeafSize (0.75f, 0.75f, 0.75f);
   vg.filter (*cloud_filtered0);
   std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

   remove_plane(cloud_filtered0, 3);

   // Remove outliers
   pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
   // build the filter
   outrem.setInputCloud(cloud_filtered0);
   outrem.setRadiusSearch(1.8);
   outrem.setMinNeighborsInRadius (6);
   // apply filter
   outrem.filter (*cloud_filtered);
   std::cout << "ROR: from" << cloud_filtered0->points.size() << " to " << cloud_filtered->points.size() << std::endl;

   pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
     tree->setInputCloud (cloud_filtered);
     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
     ec.setClusterTolerance (10); 
     ec.setMinClusterSize (100);
     ec.setMaxClusterSize (250000);
     ec.setSearchMethod(tree);
     ec.setInputCloud(cloud_filtered);
     ec.extract (cluster_indices);

     std::cout << "Found " << cluster_indices.size() << " clusters" << std::endl;
    
     int j = 0;
     SegResult maxResult;
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
       pcl::ExtractIndices<pcl::PointXYZRGBNormal> eifilter(true); // Initializing with true will allow us to extract the removed indices
       pcl::PointIndices::Ptr indicesPtr (new pcl::PointIndices());
       indicesPtr->indices = it->indices;

       eifilter.setInputCloud(cloud_filtered);
       eifilter.setIndices(indicesPtr);
       eifilter.filter(*cluster);

       SegResult res;
       find_sphere_and_mark(j, cluster, res); 
       if(maxResult.is_empty() || maxResult.size() < res.size()) {
         maxResult = res;
         maxResult.clusterInd = indicesPtr;
         std::cout << "Found new maxResult... maxResult.size = " << maxResult.size() << std::endl;
       }

       //colorPoints(cloud_filtered, *it, j);

       j++;
     }

     if(maxResult.is_empty()) {
       std::cout << "Sphere not found" << std::endl;
     } else {
       std::cout << "Sphere detected of points: " << maxResult.size() << std::endl << "Coef: " <<
         *(maxResult.coef) << std::endl;

       pcl::PointIndices pi;
       extract_indices(maxResult.ind->indices, maxResult.clusterInd->indices, pi.indices);
       colorPoints(cloud_filtered, pi, 1);

       result = maxResult;
     }


   pcl::io::savePLYFile(outFile, *cloud_filtered);

   std::cerr << "Saved " << cloud->points.size () << " data points to "<< outFile << std::endl;
}
