#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::literals::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr mapping_vis (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    auto pos_x = -10;
    auto pos_y = 40;
    auto pos_z = 5;
    auto view_x = 30;
    auto view_y = -5;
    auto view_z = -2;
    auto up_x = -10;
    auto up_y = 40;
    auto up_z = 100;
    viewer->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z, 0);
    return (viewer);
}

int main () {
  pcl::PointCloud<pcl::PointXYZI>::Ptr display_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<boost::filesystem::path> dir_files;
  std::copy(boost::filesystem::directory_iterator("westmead_pcd"), boost::filesystem::directory_iterator(), std::back_inserter(dir_files));
  std::sort(dir_files.begin(), dir_files.end());
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = mapping_vis(display_cloud);
  for (const boost::filesystem::path & filename : dir_files) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename.string(), *cloud) == -1) {
      PCL_ERROR ("Couldn't read pcd file \n");
      return (-1);
    }
    std::cout << filename.string() << std::endl;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud2, indices); // this shouldn't affect anything but doing it anyway as it will make it easier to process data later

    pcl::PassThrough<pcl::PointXYZI> x_filter; 
    x_filter.setInputCloud(cloud2);
    x_filter.setFilterFieldName("x");
    x_filter.setFilterLimits(0.0, 15);
    x_filter.filter(*cloud3);

    pcl::PassThrough<pcl::PointXYZI> z_filter;
    z_filter.setInputCloud(cloud3);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(-1, 3);
    z_filter.filter(*cloud4);

    pcl::PassThrough<pcl::PointXYZI> y_filter;
    z_filter.setInputCloud(cloud4);
    z_filter.setFilterFieldName("y");
    z_filter.setFilterLimits(-10, 10);
    z_filter.filter(*cloud5);

    viewer->updatePointCloud<pcl::PointXYZI>(cloud5, "sample cloud");
    viewer->spinOnce(50);
  }
  // std::cout << "Loaded "
  //           << cloud->width * cloud->height
  //           << " data points from test_pcd.pcd with the following fields: "
  //           << std::endl;
  // for (const auto& point: *cloud2)
  //   std::cout << "    " << point.x
  //             << " "    << point.y
  //             << " "    << point.z
  //             << " "    << point.intensity << std::endl;
  return (0);
}
