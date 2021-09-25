cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Debug)
project(pcd_read)

find_package(Eigen3 3.3 REQUIRED)

find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(LibArchive 3.2.2 REQUIRED)
link_directories(${LibArchive_INCLUDE_DIRS})


add_executable (test_visualise test_visualise.cpp)


target_link_libraries (test_visualise Eigen3::Eigen ${LibArchive_LIBRARIES} ${PCL_LIBRARIES})
