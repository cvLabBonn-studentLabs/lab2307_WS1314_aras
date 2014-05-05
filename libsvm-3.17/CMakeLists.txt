cmake_minimum_required(VERSION 2.8.3)

project(LibSVM)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

set(SRC_FILES svm.cpp)

include_directories( ${CMAKE_INCLUDE_PATH} )

add_library(svm ${SRC_FILES})
