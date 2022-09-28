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

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_read src/pcd_read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})

add_executable (pcd_write src/pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (rot_and_trans src/rot_and_trans.cpp)
target_link_libraries (rot_and_trans ${PCL_LIBRARIES})

add_executable (simple_visual src/simple_visual.cpp)
target_link_libraries (simple_visual ${PCL_LIBRARIES})

add_executable (pcl_visualizer_demo src/pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})