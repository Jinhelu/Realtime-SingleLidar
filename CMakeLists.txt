cmake_minimum_required(VERSION 3.5.0)
project(RealtimeLidar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)
set(CMAKE_BUILD_TYPE "release")
##set(CMAKE_BUILD_TYPE "debug")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 获取第三方库
FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#[[
# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )
]]

#[[
add_library(RSDecoder SHARED RSDecoder/rslidar_decoder.hpp RSDecoder/rslidar_packet.h)
target_link_libraries(RSDecoder
        ${PCL_LIBRARIES})
        ]]

##target_include_directories(RSDecoder PUBLIC RSDecoder)

######################################  PointCloudManage库  #############################################
#[[add_library(PointCloudManage SHARED src/PointCloudManage.cpp)
install(TARGETS PointCloudManage
        LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/../lib)
target_link_libraries(PointCloudManage
        ${CERES_LIBRARIES}
        )
]]

######################################  grid_manage库  #############################################
#[[add_library(grid_manage SHARED src/grid_manage.cpp)
install(TARGETS grid_manage
        LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/../lib)
]]



#####################################  IMU库  ##############################################
#[[
#头文件
include_directories(
        XSenseDrive/lib/xspublic
)
#库文件
link_directories(
        XSenseDrive/lib/xspublic/xscontroller
        XSenseDrive/lib/xspublic/xscommon
        XSenseDrive/lib/xspublic/xstypes
)
add_library(imu_get_data src/imu_get_data.cpp)

target_link_libraries(imu_get_data
        xscontroller
        xscommon
        xstypes
        rt
        dl
        )
]]
###################################################################################

include_directories(${PROJECT_BINARY_DIR}/../src)
link_directories(${PROJECT_BINARY_DIR}/../lib)

##TODO:增加对RSDecoder包的引用

set(GROUNDSEG_FILES groundSeg/ground_segmentation.cpp groundSeg/segment.cpp groundSeg/bin.cpp)
set(SOURCE_FILES src/main.cpp src/lidarMapManage.cpp src/lidarMapManage.h RSDecoder/input.cpp)
set(PARAM_FILES paramFile/paramParse.cpp)
set(VISUAL_FILES visualization/visualize.cpp)
set(OBJDET_FILES objDetect/objDetect.cpp)
add_executable(main ${SOURCE_FILES} ${GROUNDSEG_FILES} ${PARAM_FILES} ${VISUAL_FILES} ${OBJDET_FILES})
target_link_libraries(main        
        pcap
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${YAML_CPP_LIBRARIES}
        ${Eigen_LIBRARIES}
        -lpthread
        -lm
)

#[[
set(SOURCE_FILES RSDecoder/input.cpp src/GetFrame_RS.cpp  src/GetFrame_Velo16.cpp
        src/grid.h src/Auxiliary.cpp src/GetFrame_Velo16.h)
add_executable(Velo16Show ${SOURCE_FILES})
target_link_libraries(Velo16Show
        PointCloudManage
        grid_manage
        PacketDriver
        PacketDecoder
        pcap
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})
]]


#[[
set(SOURCE_FILES src/0_grid.cpp RSDecoder/input.cpp src/PointCloudManage.cpp
        src/GetFrame_RS.cpp src/grid_manage.cpp)
add_executable(grid ${SOURCE_FILES})
target_link_libraries(grid
        pcap
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})
]]
