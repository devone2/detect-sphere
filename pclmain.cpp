#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include "common.h"


int main(int argc, char** argv) {

   std::cout << "Loading plt file: " << "\n";

   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>),  cloud_f (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
;

   std::string file = "/home/controller/ceres/detect-sphere-build/img/PhoFrame(0002).ply";
   //std::string file = "/home/controller/ceres/detect-sphere-build/img/less5.ply";
   //std::string file = "/home/controller/ceres/detect-sphere-build/img/ball.ply";
   pcl::io::loadPLYFile(file, *cloud);

   // Create the filtering object: downsample the dataset using a leaf size of 1cm
   pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
   vg.setInputCloud(cloud);
   vg.setLeafSize (1.f, 1.f, 1.f);
   vg.filter (*cloud_filtered);
   std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
   std::cout << "Loaded plt file: " << "\n"
             << "Width: " << cloud->width << "\n";
   pcl::PointCloud<pcl::PointXYZRGBNormal> &c = *cloud;
   showStats(*cloud);

   int all_points = cloud_filtered->points.size();
   int t = 0;
   pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
   pcl::PointIndices::Ptr indx (new pcl::PointIndices ());
   while(cloud_filtered->points.size() > 0.1* all_points) {
     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

     detect_plane(cloud_filtered, *indx, coefficients);
     int last_size = cloud_filtered->points.size(); 
     if(indx->indices.size() == 0) {
       cout << t << ". No more planes found";
       break;
     } else {
       std::cout << t << ". Found another plane with points count:" << indx->indices.size() << std::endl;

       extract.setInputCloud(cloud_filtered);
       extract.setIndices(indx);
       extract.setNegative(true);
       extract.filter(*cloud_f);
       cloud_filtered.swap(cloud_f);

       double part = (double)indx->indices.size() / (double)last_size;
       if(part < 0.05) {
         std::cout << "Removed new plane and quiting plane search. Last plane was less than 10% of cloud. Remaining points "<< cloud_filtered->points.size() << std::endl;
         break;
       }

       std::cout << "Removed new plane and continue. Remaining points: "<< cloud_filtered->points.size() << "("<< part*100 <<"%)" << std::endl;


     }
 
     t++;
   }

   std::cout << "Planes removed. Going to find sphere..." << std::endl;



   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> segmentation;

   segmentation.setInputCloud(cloud_filtered);
   segmentation.setInputNormals(cloud_filtered);
   segmentation.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
   segmentation.setMethodType(pcl::SAC_RANSAC);
   segmentation.setDistanceThreshold(2.5);
   segmentation.setNormalDistanceWeight(0.1);
   segmentation.setOptimizeCoefficients(true);
   segmentation.setRadiusLimits(19.5, 20.7);
   segmentation.setEpsAngle(10 / (180/3.141592654));
   segmentation.setMaxIterations(500000000);

   pcl::PointIndices inlierIndices;
   segmentation.segment(inlierIndices, *coefficients);

   if (inlierIndices.indices.size() == 0)
      cout << "RANSAC nothing found" << "\n";
    else
    {
      cout << "RANSAC found shape with [%d] points:" << inlierIndices.indices.size() << "\n";
      //for (int c=0; c<coefficients->values.size(); ++c)
      //    ROS_INFO("Coeff %d = [%f]", (int)c+1, (float)coefficients->values[c]);

      // mark the found inliers in green
      for (int m=0; m<inlierIndices.indices.size(); ++m)
      {
          cloud_filtered->points[inlierIndices.indices[m]].r = 255;
          cloud_filtered->points[inlierIndices.indices[m]].g = 0;
          cloud_filtered->points[inlierIndices.indices[m]].b = 0;
      }

      std::cout << "Model coefficient: " << *coefficients << std::endl;
    }
   
   pcl::io::savePLYFile("test_ply.ply", *cloud_filtered);

   std::cerr << "Saved " << cloud->points.size () << " data points to test_ply.ply." << std::endl;
}

