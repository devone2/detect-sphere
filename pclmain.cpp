#include <iostream>

#include<pcl/io/ply_io.h>

int main(int argc, char** argv) {

   std::cout << "Loading plt file: " << "\n";

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ);
   pcl::PLYReader Reader;

   std::string file("img/PhoFrame(0000).ply");
   Reader.read(file, *cloud);

   std::cout << "Loaded plt file: " << file <<"\n"
           //  << "Points in cloud: " << cloud->points.size() << "\n";
}
