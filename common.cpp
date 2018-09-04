
#include "common.h"

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
