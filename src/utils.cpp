#include "utils.h"
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

void
 plot_point_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)

 {
   
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }

 }