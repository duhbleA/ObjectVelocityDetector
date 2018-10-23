cmake_minimum_required(VERSION 3.5)
project(ObjectVelocityDetector)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
set(CMAKE_CXX_STANDARD 11)
set(PYTHON_EXECUTABLE "/usr/bin/python")

find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(NumPy REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(${PYTHON_INCLUDE_DIRS} ${PYTHON_NUMPY_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/include/conversions.h  ${OpenCV_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS} ${PCAP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(ObjectVelocityDetector src/main.cpp src/conversion.cpp)
target_link_libraries(ObjectVelocityDetector "${PYTHON_LIBRARIES}" ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
