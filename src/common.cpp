
#include "common.h"

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

void colorPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,const pcl::PointIndices& indices, int colorIndex) {
  for (std::vector<int>::const_iterator pit = indices.indices.begin (); pit != indices.indices.end (); ++pit) {
    uint32_t c = colors[colorIndex % (colors.size())];
    cloud->points[*pit].r = c >> 16 & 0x0000ff;
    cloud->points[*pit].g = c >> 8 & 0x0000ff;
    cloud->points[*pit].b = c & 0x0000ff;
  }
}

void detect_plane(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,pcl::PointIndices& indices, pcl::ModelCoefficients::Ptr& coefficients) {
  
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> segmentation;

  segmentation.setInputCloud(cloud);
  segmentation.setInputNormals(cloud);
  segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(1.5);
  segmentation.setNormalDistanceWeight(0.1);
  segmentation.setOptimizeCoefficients(true);
  //  segmentation.setRadiusLimits(18,22);
  segmentation.setEpsAngle(1 / (180/3.141592654));
  segmentation.setMaxIterations(10000);

  segmentation.segment(indices, *coefficients);
}

bool remove_plane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, int max)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
     pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
     pcl::PointIndices::Ptr indx (new pcl::PointIndices ());

     for(int t=0;t<max;t++) {
      detect_plane(cloud, *indx, coefficients);
      int last_size = cloud->points.size(); 
      if(indx->indices.size() == 0) {
        std::cout << t << ". No more planes found";
        return false;
      } else {
        std::cout << t << ". Found another plane with points count:" << indx->indices.size() << std::endl;

        extract.setInputCloud(cloud);
        extract.setIndices(indx);
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud.swap(cloud_f);

        double part = (double)indx->indices.size() / (double)last_size;
        std::cout << "Removed new plane and continue. Remaining points: "<< cloud->points.size() << "("<< part*100 <<"%)" << std::endl;
      }
     }
     return true;
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
