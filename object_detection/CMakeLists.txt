cmake_minimum_required(VERSION 3.1.0)

project(cluster_extraction)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# set(realsense2_DIR /usr/local/lib/cmake/realsense2)

find_package(PCL 1.2 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenGL)

find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
PATHS /usr/X11R6/include
/usr/include/X11
/opt/graphics/OpenGL/include
/opt/graphics/OpenGL/contrib/libglfw
/usr/local/include
/usr/include/GL
/usr/include
)
find_library(GLFW_LIBRARIES NAMES glfw glfw3
PATHS /usr/lib64
/usr/lib
/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
/usr/local/lib64
/usr/local/lib
/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
/usr/X11R6/lib
)

# add_dependencies(object_tracking ${catkin_EXPORTED_TARGETS})
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIR}
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
    src/cluster_extraction.cpp
)
target_link_libraries (test_realsense 
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES} 
)

add_executable (test_static 
    src/test_static.cpp
    src/cluster_extraction.cpp
)
target_link_libraries (test_static 
${PCL_LIBRARIES})
