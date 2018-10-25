cmake_minimum_required(VERSION 3.5)
project(vpl_no_view)

#add_subdirectory(src)
#include_directory("../../src/transformhelper.h")
#add_executable(foo ${SRC_LIST})

set(CMAKE_CXX_STANDARD 11)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${PCAP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(vpl_no_view main.cpp)
target_link_libraries(vpl_no_view ${PCL_LIBRARIES})
