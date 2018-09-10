# detect-sphere
Detecting position of sphere of specific radius in point cloud

# Build
Requires pcl library. Simplest way to build is using provided Dockerfile and build and execute app in container. See bin/dorun script for example
when in container build with cmake

# Run
```
./cluster input_dir output_dir
```
Program expects only *.ply files in input_dir. It will produce filtered *.ply files with marked detected ball with yellow color. It will write sphere parameters in results.csv.
PCL ply file reader cannot handle line
```
obj_info Photoneo PLY PointCloud ( Width = 2064; Height = 1544)
```
so one has to remove it before executing ./cluster. Do not forget to edit file in binary mode.

# Algorithm
The purpose is detecting ping-pong ball held in robot manipulator. Basic algorithm used is RANSAC for sphere model. But this algorithm
often fails when there are too many outliers as is it case with whole scene scans. 

Processing of cloud:
- downsampling with voxel filter
- detecting 3 biggest planes with RANSAc
- filter out outlier with not enough neighbours (RadiusOutlierRemoval)
- clustering with EuclideanClusterExtraction with realative big tolerance (10 mm) to cluster ball in single cluster even though it is split to two parts with oclusion.
- run RANSAC Sphere with <20,21> radius constraint for each cluster
- select the sphere with the biggest number of inliers

# Results
| Detected            |       |        |          |          |         | Labeled |          |          |         |                   |                   | 
|---------------------|-------|--------|----------|----------|---------|---------|----------|----------|---------|-------------------|-------------------| 
| file                | found | points | x        | y        | z       | r       | x        | y        | z       | L2^2              | L2                | 
| PhoFrame(0000).ply  | Y     | 1235   | 180.335  | -90.818  | 735.674 | 20.2281 | -116.398 | -102.548 | 786.796 | 90801.525073      | 301.332914022017  | 
| PhoFrame(0001).ply  | Y     | 1832   | -122.827 | -170.521 | 706.677 | 20.3517 | -122.727 | -170.4   | 706.62  | 0.027889999999996 | 0.167002993985126 | 
| PhoFrame(0002).ply  | Y     | 1731   | -127.66  | 10.1173  | 710.454 | 20.2029 | -127.608 | 10.154   | 710.52  | 0.008406890000003 | 0.091689094226104 | 
| PhoFrame(0003).ply  | Y     | 1424   | 29.5975  | 14.8242  | 681.291 | 20.2559 | 29.619   | 14.872   | 681.39  | 0.012548089999987 | 0.112018257440413 | 
| PhoFrame(0004).ply  | Y     | 1234   | 175.702  | 19.3276  | 654.614 | 20.2183 |          |          |         | 459764.23792176   | 678.059169926755  | 
| PhoFrame(0005).ply  | Y     | 1538   | 168.917  | 8.86532  | 654.183 | 20.3445 | 168.906  | 8.919    | 654.248 | 0.007227542400007 | 0.085014953978738 | 
| PhoFrame(0006).ply  | Y     | 1652   | 167.493  | -85.7606 | 652.929 | 20.2935 | 167.474  | -85.702  | 653.016 | 0.011363959999998 | 0.106601876156089 | 
| PhoFrame(0007).ply  | Y     | 1790   | 170.417  | -197.055 | 650.452 | 20.2861 | 170.408  | -197.048 | 650.511 | 0.003610999999997 | 0.060091596750267 | 
| PhoFrame(0008).ply  | Y     | 1685   | -7.79071 | -184.254 | 679.652 | 20.1222 | -7.643   | -184.331 | 679.379 | 0.102276244100013 | 0.319806572946857 | 
| PhoFrame(0009).ply  | Y     | 1477   | 21.7495  | -96.4661 | 702.727 | 20.2413 | 21.744   | -96.474  | 702.877 | 0.022592659999993 | 0.150308549324359 | 
| PhoFrame(00010).ply | Y     | 1742   | -54.4977 | -70.1152 | 776.216 | 20.2156 | -54.475  | -70.082  | 776.269 | 0.00442653        | 0.066532172668569 | 


If we exclude obviously bad match (0000, 0004) than root mean square error 0.1mm.

# Snapshots
