#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include<pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

void showStats(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);

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
   
   segmentation.setInputCloud(cloud);
   segmentation.setInputNormals(cloud);
   segmentation.setModelType(pcl::SACMODEL_NORMAL_PLAIN);
   segmentation.setMethodType(pcl::SAC_RANSAC);
   segmentation.setDistanceThreshold(1);
   segmentation.setNormalDistanceWeight(0.1);
   segmentation.setOptimizeCoefficients(true);
   segmentation.setRadiusLimits(15,25);
   segmentation.setEpsAngle(15 / (180/3.141592654));
   segmentation.setMaxIterations(100000);

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

void showStats(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud) {
    int minx = -1;
    int miny = -1;
    int minz = -1;
    for(int i=0;i<cloud.points.size();i++) {
        pcl::PointXYZRGBNormal& p = cloud.points[i];
        if(minx == -1 || minx > p.x) {
            minx = p.x;
        }
        if(miny == -1 || miny > p.y) {
            miny = p.y;
        }
        if(minz == -1 || minz > p.z) {
            minz = p.z;
        }
    }

    int maxx = -1;
    int maxy = -1;
    int maxz = -1;
    for(int i=0;i<cloud.points.size();i++) {
        pcl::PointXYZRGBNormal& p = cloud.points[i];
        if(maxx == -1 || maxx < p.x) {
            maxx = p.x;
        }
        if(maxy == -1 || maxy < p.y) {
            maxy = p.y;
        }
        if(maxz == -1 || maxz < p.z) {
            maxz = p.z;
        }
    }
    int dx = maxx - minx;
    int dy = maxy - miny;
    int dz = maxz - minz;

    std::cout << "dx: " << dx << std::endl
         << "dy: " << dy << std::endl
         << "dz: " << dz << std::endl;

    int v = dx*dy*dz;
    double precision = std::pow((double)v / cloud.points.size(), 1/3.);

    std::cout << "Precission: " << (double)v / (double)cloud.points.size() << std::endl;

    std::cout << "Precission: " << precision << std::endl;
    std::cout << "Points: " << cloud.points.size() << std::endl;
    
}
