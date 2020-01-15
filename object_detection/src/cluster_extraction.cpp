#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

#include "time.h"
#include "cluster_extraction.h"

#define MIN_HEIGHT -100.0
#define MAX_HEIGHT 100.0

void filter_voxel_height(const pcl_ptr & orig, pcl_ptr & cloud_filtered) {
  pcl_ptr cloud_vg (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (orig);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_vg);
  
  // Create the filtering object: remove ground plane by thresholding height
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_vg);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (MIN_HEIGHT, MAX_HEIGHT);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
}

void find_clusters(pcl_ptr & cloud_filtered, std::vector<pcl::PointIndices> & cluster_groupings){

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl_ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.08); // 2cm
  ec.setMinClusterSize (20000);
  ec.setMaxClusterSize (50000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_groupings);
}

void visualize_pc_clusters(std::vector<pcl::PointIndices> & cluster_groupings,
    const pcl_ptr & cloud) {

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  srand(time(NULL));
  int j = 0;
  int r, g, b;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_groupings.begin (); it != cluster_groupings.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j;
    j++;

    r = rand() % 255;
    g = rand() % 255;
    b = rand() % 255;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
      cloud_cluster, r, g, b);
    viewer.addPointCloud (cloud_cluster, single_color, ss.str());// note that before it was showCloud
    
  }
  viewer.spin();
}