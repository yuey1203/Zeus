#include "time.h"
#include <pcl/io/pcd_io.h>
#include "cluster_extraction.h"

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform Application
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>            // std::min, std::max


pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}


int main(int argc, char * argv[])
{
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    bool running = true;

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    while (running) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            color = frames.get_infrared_frame();

        // Tell pointcloud object to map to this color frame
        // may not be necessary if don't use color
        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        pcl_ptr cloud = points_to_pcl(points);
        pcl_ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

        // filter_voxel_height(cloud, cloud_filtered);
        // std::vector<pcl::PointIndices> cluster_groupings;
        // find_clusters(cloud_filtered, cluster_groupings);
        // pcl::PCDWriter writer;
        // writer.write<pcl::PointXYZ> ("test_pcd.pcd", *cloud, false);
          
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
            cloud, 255, 255, 255);
        viewer.addPointCloud (cloud, single_color, "View PC");// note that before it was showCloud
        viewer.spin();
        viewer.removePointCloud ("View PC");
        // visualize_pc_clusters(cluster_groupings, cloud_filtered);
    }

    return EXIT_SUCCESS;
}
