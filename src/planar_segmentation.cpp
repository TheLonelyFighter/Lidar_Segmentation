#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>




void
 plot_point_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string plot_name)
 {
   
   pcl::visualization::CloudViewer viewer (plot_name);
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
 }


int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);


  //For some reason which I am not aware of atm, this code works best for rosbag_5.pcd
  //This segmentation alg is very simple and cannot differentiate between different ground elevation
  //
  // Load the pcd file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../pcd_files/rosbag_5.pcd", *cloud) == -1) //* load the file

  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  std::cout << "This is a test" << std::endl;

  std::cout << "Point cloud BEFORE segmentation" << std::endl;
  plot_point_cloud(cloud, "Point cloud BEFORE segmentation");

  // Set a few outliers
  // (*cloud)[0].z = 2.0;
  // (*cloud)[3].z = -2.0;
  // (*cloud)[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  // for (const auto& point: *cloud)
  //   std::cerr << "    " << point.x << " "
  //                       << point.y << " "
  //                       << point.z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.5);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  pcl::ModelOutlierRemoval<pcl::PointXYZ> plane_filter;
  plane_filter.setModelCoefficients (*coefficients);
  plane_filter.setThreshold (0.5);
  plane_filter.setModelType (pcl::SACMODEL_PLANE);
  plane_filter.setInputCloud (cloud);
  plane_filter.filter (*ground_plane); //contains oly the segmented ground plane

  plane_filter.setNegative (true);
  plane_filter.filter(*objects); //contains only the objects

  
	plot_point_cloud(objects, "Objects");
  
  plot_point_cloud(ground_plane, "Ground Plane");

  // Visualization
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Segmented point cloud");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 128,0,0);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (ground_plane, 128, 0, 128); // Red
  viewer.addPointCloud (ground_plane, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  viewer.saveCameraParameters("camera_params.txt");
  viewer.setCameraPosition(-200, 0, 0, 0, 0, 0, 0); // parameters measured in meters
  viewer.saveScreenshot("../results/screenshot_seg.png");
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return (0);
}

