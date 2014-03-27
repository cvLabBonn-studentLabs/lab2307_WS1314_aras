cmake_minimum_required(VERSION 2.8.3)

project(POSE_ESTIMATION)

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization search segmentation features filters surface kdtree)

################################
# OpticalFlow Library
################################
find_library(LDOF ldof PATHS tracking)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
         
set(SRC_FILES src/keyframe.cpp
			  src/data_io_backend.cpp
			  src/data_io.cpp
              src/cornell/readData.cpp
              src/cornell/backend.cpp
              src/ethz/backend.cpp
              src/stanford/backend.cpp
              src/bonn/backend.cpp
              src/main.cpp
              src/segmentation.cpp
              src/classifier_svm.cpp
              src/timer.cpp
              src/opflow.cpp
              src/opflow_tensor.cpp
              )

set(MESH_FILES src/mesher.cpp src/timer.cpp)

set(LIBRARIES opencv_core
		opencv_highgui
		pcl_visualization
		boost_system
		boost_filesystem
		opencv_imgproc
		pcl_common
		pcl_search
		pcl_segmentation
		pcl_features
		pcl_filters
		pcl_io
		pcl_surface
		pcl_kdtree
		gtest
		svm
		${LDOF})

include_directories(include
	${PCL_INCLUDE_DIRS}
	${CMAKE_INCLUDE_PATH}
	libsvm-3.17
	tracking
)

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/lib)

add_library(mesh ${MESH_FILES})
target_link_libraries(mesh ${LIBRARIES})

add_executable(estimate_pose ${SRC_FILES})

target_link_libraries(estimate_pose mesh ${LIBRARIES})

################################
# LibSVM
################################
ADD_SUBDIRECTORY (libsvm-3.17)

################################
# GTest
################################
ADD_SUBDIRECTORY (gtest-1.7.0)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

################################
# Unit Tests
################################
# Add test cpp file
add_executable( runUnitTests tests/mesh_tests.cpp )
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTests mesh gtest gtest_main )
add_test( runUnitTests runUnitTests )
