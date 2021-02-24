cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


project(whySegmentation)  


find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories("/Users/zhangmeng/Desktop/Source/include")

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable (whySegmentation main.cpp myself.cpp)  

target_link_libraries (whySegmentation ${PCL_LIBRARIES}) 