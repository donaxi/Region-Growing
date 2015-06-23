cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include_directories(BEFORE ${PCL_INCLUDE_DIRS})
include_directories(/usr/include/vtk-5.8)
#include_directories(/usr/include/pcl-1.7)
#find_package(PCL 1.7 REQUIRED)
#include_directories(BEFORE ${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#if (NOT PCL_FOUND)
    #MESSAGE(FATAL_ERROR "PCL not found.\n")
#endif (NOT PCL_FOUND)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#SET_TARGET_PROPERTIES(color PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(color src/color.cpp)
rosbuild_add_executable(prueba src/prueba.cpp)
rosbuild_add_executable(correspondence src/correspondence.cpp) 
target_link_libraries(color ${PCL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
target_link_libraries(correspondence ${PCL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
#target_link_libraries(example ${PROJECT_NAME})
#TARGET_LINK_LIBRARIES(ATFOR ${OpenCV_LIBS} curl )
