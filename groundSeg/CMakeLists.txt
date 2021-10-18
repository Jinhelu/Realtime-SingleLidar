cmake_minimum_required(VERSION 2.8)
project(ground_seg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set (CMAKE_CXX_FLAGS "-O2")

set(CMAKE_BUILD_TYPE Release)

include_directories(
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

add_executable(test_seg
    main.cpp
	ground_segmentation.cpp
	segment.cpp
	bin.cpp)

target_link_libraries(test_seg 
	${Eigen_LIBRARIES}
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	-lpthread
	-lm
	)
