cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_extraction)

find_package(PCL 1.2 REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (object_tracking 
    src/object_tracking.cpp
    src/cluster_extraction.cpp
)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

add_executable (test_static 
    src/test_static.cpp
    src/cluster_extraction.cpp
)
target_link_libraries (test_static ${PCL_LIBRARIES})

