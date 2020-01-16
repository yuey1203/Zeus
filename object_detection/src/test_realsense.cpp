// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include "cluster_extraction.h"

#include <algorithm>            // std::min, std::max

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

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
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    texture color_image;     // Helpers for renderig images

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR);
    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    while (app) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            color = frames.get_infrared_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        pcl_ptr cloud = points_to_pcl(points);
        pcl_ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

        // filter_voxel_height(cloud, cloud_filtered);
        std::vector<pcl::PointIndices> cluster_groupings;
        find_clusters(cloud, cluster_groupings);

        double cx, cy, cz;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_groupings.begin (); 
            it != cluster_groupings.end (); ++it)
        {
            cx = 0; cy = 0; cz = 0;
            for (std::vector<int>::const_iterator pit = it->indices.begin (); 
                    pit != it->indices.end (); ++pit) 
            {
                cx += cloud->points[*pit].x;
                cy += cloud->points[*pit].y;
                cz += cloud->points[*pit].z;
            }

            cx /= cloud->points.size();
            cy /= cloud->points.size();
            cz /= cloud->points.size();

        }
        

        // Upload the color frame to OpenGL
        // app_state.tex.upload(color);

        // Draw the pointcloud
        // draw_pointcloud(app.width(), app.height(), app_state, points);

        color_image.render(color, { 0, 0, app.width(), app.height() });


        // pcl::PCDWriter writer;
        // writer.write<pcl::PointXYZ> ("test_pcd.pcd", *cloud, false);
          
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
        //     cloud, 255, 255, 255);
        // printf("Number of points: %ld\n", cloud->points.size());
        // viewer.addPointCloud (cloud, single_color, "View PC");// note that before it was showCloud
        // viewer.spin();
        // viewer.removePointCloud ("View PC");

        std::stringstream ss; 
        ss << (int)cx << ", " << (int)cy << ", " << (int)cz;
        std::string label = ss.str();
        
        draw_text((int)cy, (int)cx, label.c_str());
    }

    return 0;
}