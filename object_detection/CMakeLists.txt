cmake_minimum_required(VERSION 3.1.0)

project(cluster_extraction)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# set(realsense2_DIR /usr/local/lib/cmake/realsense2)

find_package(PCL 1.2 REQUIRED)
find_package(realsense2 REQUIRED)
# add_dependencies(object_tracking ${catkin_EXPORTED_TARGETS})
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
target_link_libraries (object_tracking 
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES}
)

add_executable (test_realsense  
    src/test_realsense.cpp
)
target_link_libraries (test_realsense 
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES}
)

add_executable (test_static 
    src/test_static.cpp
    src/cluster_extraction.cpp
)
target_link_libraries (test_static 
${PCL_LIBRARIES})
