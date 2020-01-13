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

int main(int argc, char* argv[]) {
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl_ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Input Cloud File: " << argv[1] << std::endl;
  reader.read (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  filter_voxel_height(cloud, cloud_filtered);

  std::vector<pcl::PointIndices> cluster_groupings;
  find_clusters(cloud_filtered, cluster_groupings);

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  srand(time(NULL));
  int j = 0;
  int r, g, b;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_groupings.begin (); it != cluster_groupings.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      
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

  return (0);
}