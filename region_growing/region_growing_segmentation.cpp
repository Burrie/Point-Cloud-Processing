#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

//Segmentation
#include <pcl/segmentation/region_growing.h>

//Filter
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>

//Visualization
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
using namespace std::chrono_literals;

int user_data = 0;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer){
  viewer.setBackgroundColor (1.0, 0.5, 1.0);
  pcl::PointXYZ o;
  o.x = 1.0;
  o.y = 0;
  o.z = 0;
  viewer.addSphere (o, 0.25, "sphere", 0);
  std::cout << "i only run once" << std::endl;
}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer) {
  static unsigned count = 0;
  std::stringstream ss;
  ss << "Once per viewer loop: " << count++;
  viewer.removeShape ("text", 0);
  viewer.addText (ss.str(), 200, 300, "text", 0);
  user_data++;
}

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

//-----------------------------------------Region Growing Segmentation--------------------------------------------------------
int main ()
{
  char* input_pointCloud_file = "/Users/xiangqiong/Desktop/DHMT/region_growing/region_growing_tutorial.pcd";

  //Get input
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (input_pointCloud_file, *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  //Process
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (200);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*cloud, *indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (100);
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" << std::endl << "cloud that belong to the first cluster:" << std::endl;

  std::size_t counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }

  std::cout << std::endl;

  //------------------------------------Extract indices to pcd file------------------------------------------------------------

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDWriter writer;

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  //Extract each cluster
  for(int i = 0; i < clusters.size(); i++)
  {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices (clusters[i]));

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "../segmented_cloud/segment" << "_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    extract.setNegative (true);
  }

  //------------------------------------ Cloud viewer -------------------------------------------------------------
  // //Show point cloud
  // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  // pcl::visualization::CloudViewer viewer ("Cluster viewer");
  // viewer.showCloud(colored_cloud);
  // viewer.runOnVisualizationThreadOnce (viewerOneOff);
  // viewer.runOnVisualizationThread (viewerPsycho);
  // while (!viewer.wasStopped ())
  // {
  //   user_data++;
  // }
  //------------------------------------- Visualizer ------------------------------------------------------------
  // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  // pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(colored_cloud);

  // while (!viewer->wasStopped ())
  // {
  //   viewer->spinOnce (100);
  //   std::this_thread::sleep_for(100ms);
  // }

  return (0);
}