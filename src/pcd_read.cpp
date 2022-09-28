#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


void
 plot_point_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
 {
   
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
 }


int
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../pcd_files/test_pcd.pcd", *cloud) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../pcd_files/rosbag_6.pcd", *cloud) == -1) //* load the file

  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  int total_points = cloud->width * cloud->height;
  std::cout << "Loaded pcd successfully " << std::endl 
            << "Width: " << (int)cloud->width << std::endl // 
            << "Height: " << (int)cloud->height << std::endl // width == 1 means unorganized point cloud dataset
            << "Total number of points: "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  int counter = 0;

  std::cout << "        x  " << "      y  " << "      z" << std::endl;
  for (const auto& point: *cloud)
  {
    
    std::cout << "    " << point.x
              << " "    << point.y
              << " "    << point.z << std::endl;
    if (counter++ > 100)
      break;
  }

  //Compute the average altitude
  float total_z = 0;
  for (const auto& point: *cloud)
  {
    total_z += point.z;
  }
  
  std::cout << "Average altitude: " << total_z / total_points << std::endl;

  if (cloud->is_dense)
    std::cout << "is_dense_true"<< std::endl; //it means there are no points with NaN values
  else
    std::cout << "is_dense_false"<< std::endl;

  if (cloud->isOrganized ())  // same as checking if height == 1
    std::cout << "is_organized_true"<< std::endl;
  else
    std::cout << "is_organized_false"<< std::endl;

  plot_point_cloud(cloud);
  
  return (0);
}
