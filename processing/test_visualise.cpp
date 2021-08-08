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
#include <pcl/segmentation/segment_differences.h>

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
  // double filter_x_min = 0.0;
  // double filter_x_max = 15;
  // double filter_z_min = -1;
  // double filter_z_max = 3;
  // double filter_y_min = -12.5;
  // double filter_y_max = 12.5;
  bool bg_filtering = false;

  double filter_x_min = -10000;
  double filter_x_max = 10000;
  double filter_z_min = -10000;
  double filter_z_max = 10000;
  double filter_y_min = -10000;
  double filter_y_max = 10000;
  pcl::PointCloud<pcl::PointXYZI>::Ptr display_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr calib_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<boost::filesystem::path> dir_files;
  std::copy(boost::filesystem::directory_iterator("westmead_pcd"), boost::filesystem::directory_iterator(), std::back_inserter(dir_files));
  std::sort(dir_files.begin(), dir_files.end());
  std::vector<boost::filesystem::path> calib_files;
  std::copy(boost::filesystem::directory_iterator("westmead_pcd/calibration"), boost::filesystem::directory_iterator(), std::back_inserter(calib_files));
  std::sort(calib_files.begin(), calib_files.end());
  // First do calibration - right now just break after reading first calibration file
  for (const boost::filesystem::path & filename : calib_files) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZI>);
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
    x_filter.setFilterLimits(filter_x_min, filter_x_max);
    x_filter.filter(*cloud3);

    pcl::PassThrough<pcl::PointXYZI> z_filter;
    z_filter.setInputCloud(cloud3);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(filter_z_min, filter_z_max);
    z_filter.filter(*cloud4);

    pcl::PassThrough<pcl::PointXYZI> y_filter;
    z_filter.setInputCloud(cloud4);
    z_filter.setFilterFieldName("y");
    z_filter.setFilterLimits(filter_y_min, filter_y_max);
    z_filter.filter(*calib_cloud);
    break;
  }
  std::cout << "====================" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = mapping_vis(display_cloud);
  viewer->addLine(pcl::PointXYZ(0,0,0), pcl::PointXYZ(300,0,0), 255, 0, 0, std::string("median_divider_line"), 0);
  for (const boost::filesystem::path & filename : dir_files) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud6 (new pcl::PointCloud<pcl::PointXYZI>);
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
    x_filter.setFilterLimits(filter_x_min, filter_x_max);
    x_filter.filter(*cloud3);

    pcl::PassThrough<pcl::PointXYZI> z_filter;
    z_filter.setInputCloud(cloud3);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(filter_z_min, filter_z_max);
    z_filter.filter(*cloud4);

    pcl::PassThrough<pcl::PointXYZI> y_filter;
    z_filter.setInputCloud(cloud4);
    z_filter.setFilterFieldName("y");
    z_filter.setFilterLimits(filter_y_min, filter_y_max);
    z_filter.filter(*cloud5);

    if (bg_filtering) {
      pcl::SegmentDifferences<pcl::PointXYZI> difference_segmenter;
      difference_segmenter.setInputCloud(cloud5);
      difference_segmenter.setTargetCloud(calib_cloud);
      difference_segmenter.setDistanceThreshold(0.5);
      difference_segmenter.segment(*cloud6);  
    }
    
    if (bg_filtering) {
      viewer->updatePointCloud<pcl::PointXYZI>(cloud6, "sample cloud");
    }
    else {
      viewer->updatePointCloud<pcl::PointXYZI>(cloud5, "sample cloud");
    }
    
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
