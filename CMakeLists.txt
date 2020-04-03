cmake_minimum_required(VERSION 2.8)

project(PCL_benchmark)
find_package(PCL 1.3 REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PCL_benchmark main.cpp)
target_link_libraries(PCL_benchmark ${PCL_LIBRARIES} benchmark)
