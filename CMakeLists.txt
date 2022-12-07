# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# project(pcd_write)

# find_package(PCL 1.2 REQUIRED)

# SET (PROJECT_SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")

# include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable (pcd_write src/pcd_write.cpp)
# target_link_libraries (pcd_write ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)


# SET (YOUR_DIRECTORY "/src")
# target_include_directories(test PRIVATE ${YOUR_DIRECTORY})

include_directories(${PCL_INCLUDE_DIRS} src)
link_directories(${PCL_LIBRARY_DIRS} src)
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_read src/pcd_read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})

# add_executable (pcd_write src/pcd_write.cpp)
# target_link_libraries (pcd_write ${PCL_LIBRARIES})

# add_executable (rot_and_trans src/rot_and_trans.cpp)
# target_link_libraries (rot_and_trans ${PCL_LIBRARIES})

# add_executable (simple_visual src/simple_visual.cpp)
# target_link_libraries (simple_visual ${PCL_LIBRARIES})

add_executable (planar_segmentation src/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

# add_executable (model_outlier_removal src/model_outlier_removal.cpp src/utils.h src/utils.cpp)
# target_link_libraries (model_outlier_removal ${PCL_LIBRARIES} )

# add_executable (range_image_creation src/range_image_creation.cpp)
# target_link_libraries (range_image_creation ${PCL_LIBRARIES})

# add_executable (range_image_visualization src/range_image_visualization.cpp)
# target_link_libraries (range_image_visualization ${PCL_LIBRARIES})

# add_executable (surface_normal src/surface_normal.cpp)
# target_link_libraries (surface_normal ${PCL_LIBRARIES})

# add_executable (kdtree_search src/kdtree_search.cpp)
# target_link_libraries (kdtree_search ${PCL_LIBRARIES})

# add_executable (octree_search src/octree_search.cpp)
# target_link_libraries (octree_search ${PCL_LIBRARIES})

# add_executable (octree_change_detection src/octree_change_detection.cpp)
# target_link_libraries (octree_change_detection ${PCL_LIBRARIES})


add_executable (supervoxel_clustering src/supervoxel_clustering.cpp)
target_link_libraries (supervoxel_clustering ${PCL_LIBRARIES})


