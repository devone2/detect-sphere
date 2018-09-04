#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>


#include "common.h"

int main(int argc, char** argv) {

   std::cout << "Loading plt file: " << "\n";

   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

   //std::string file = "/home/controller/ceres/detect-sphere-build/img/PhoFrame(0007).ply";
   std::string file = "/home/controller/ceres/detect-sphere-build/img/less5.ply";
   //std::string file = "/home/controller/ceres/detect-sphere-build/img/ball.ply";
   pcl::io::loadPLYFile(file, *cloud);

   /*
   pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
   sor.setInputCloud (cloud);
   sor.setLeafSize (1, 1, 1);
   sor.filter (*cloud_filtered);
*/


   /*
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(cloud, 0, 255, 0);
   viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, "sample cloud");
   //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   viewer->addCoordinateSystem (1.0);

   while (!viewer->wasStopped ())
   {
     viewer->spinOnce (100);
     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
*/
/*   ;

   pcl::PCLPointCloud2 blob;
   Eigen::Vector4f origin;
   Eigen::Quaternionf orientation;
   int version;
   int data_type;
   unsigned int data_idx;

   Reader.readHeader(file, blob, origin, orientation, version, data_type, data_idx);

   std::cout << "Header read - version:" << version << "\n";


   Reader.read(file, *cloud);
*/
   std::cout << "Loaded plt file: " << "\n"
             << "Width: " << cloud->width << "\n";
   pcl::PointCloud<pcl::PointXYZRGBNormal> &c = *cloud;


   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> segmentation;

  /*
   for(int i=0;i<c.size();i++) {
       cout << i << ". " << c[i].x <<","<<c[i].y << "," << c[i].z
            << " -> N: " << c[i].normal_x << ","<< c[i].normal_y << ","<< c[i].normal_z << std::endl;
   }*/
   showStats(*cloud);
   //showStats(*cloud_filtered);

   /*
   pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::KdTree<pcl::PointXYZRGBNormal>);
     tree->setInputCloud (*cloud);

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
     ec.setClusterTolerance (0.02); // 2cm
     ec.setMinClusterSize (100);
     ec.setMaxClusterSize (25000);
     ec.setSearchMethod (tree);
     ec.setInputCloud (*cloud);
     ec.extract (cluster_indices);

     int j = 0;
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {

           cloud->points[*pit].r = (100 + (j*30)) % 256;
           cloud->points[*pit].g = 0;
           cloud->points[*pit].b = 0;
       }

       std::cout << j << ". pointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

       j++;
     }
*/




   segmentation.setInputCloud(cloud);
   segmentation.setInputNormals(cloud);
   segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE);
   segmentation.setMethodType(pcl::SAC_RANSAC);
   segmentation.setDistanceThreshold(1);
   segmentation.setNormalDistanceWeight(0.5);
   segmentation.setOptimizeCoefficients(true);
   segmentation.setRadiusLimits(15,25);
   segmentation.setEpsAngle(1 / (180/3.141592654));
   segmentation.setMaxIterations(10000);

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
          cloud->points[inlierIndices.indices[m]].r = 255;
          cloud->points[inlierIndices.indices[m]].g = 0;
          cloud->points[inlierIndices.indices[m]].b = 0;
      }

      std::cout << "Model coefficient: " << *coefficients << std::endl;
    }

   pcl::io::savePLYFile("test_ply.ply", *cloud);

   std::cerr << "Saved " << cloud->points.size () << " data points to test_ply.ply." << std::endl;
}
