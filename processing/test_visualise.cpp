#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

using namespace std::literals::chrono_literals;

# define M_PI           3.14159265358979323846  /* pi */

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

bool customRegionGrowing(const pcl::PointXYZINormal& a, const pcl::PointXYZINormal& b, float squared_d) {
  return true;
}

int main () {
  double filter_x_min = 0.0;
  double filter_x_max = 15;
  double filter_z_min = -1;
  double filter_z_max = 3;
  double filter_y_min = -12.5;
  double filter_y_max = 12.5;
  bool bg_filtering = true;
  bool pass_through_filtering = false;
  bool box_filtering = true;

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
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughIn (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr bgBetter (new pcl::PointCloud<pcl::PointXYZI>);


    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename.string(), *cloud) == -1) {
      PCL_ERROR ("Couldn't read pcd file \n");
      return (-1);
    }
    std::cout << filename.string() << std::endl;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *passThroughIn, indices); // this shouldn't affect anything but doing it anyway as it will make it easier to process data later

    if (pass_through_filtering){
      pcl::PassThrough<pcl::PointXYZI> x_filter; 
      x_filter.setInputCloud(passThroughIn);
      x_filter.setFilterFieldName("x");
      x_filter.setFilterLimits(filter_x_min, filter_x_max);
      x_filter.filter(*passThroughBetter);

      passThroughIn->clear();

      pcl::PassThrough<pcl::PointXYZI> z_filter;
      z_filter.setInputCloud(passThroughBetter);
      z_filter.setFilterFieldName("z");
      z_filter.setFilterLimits(filter_z_min, filter_z_max);
      z_filter.filter(*passThroughIn);

      passThroughBetter->clear();

      pcl::PassThrough<pcl::PointXYZI> y_filter;
      z_filter.setInputCloud(passThroughIn);
      z_filter.setFilterFieldName("y");
      z_filter.setFilterLimits(filter_y_min, filter_y_max);
      z_filter.filter(*passThroughBetter);
    }
    else {
      pcl::copyPointCloud(*passThroughIn, *passThroughBetter);
    }

    if (box_filtering){
      pcl::CropBox<pcl::PointXYZI> box_filter;
      std::vector<int> indices2;
      box_filter.setInputCloud(passThroughBetter);
      Eigen::Vector4f min_pt (0.0f, 0.0f, -20.0f, 1);
      Eigen::Vector4f max_pt (7.7f, 21.341f, 20.0f, 1);
      box_filter.setMin (min_pt);
      box_filter.setMax (max_pt);
      box_filter.setTransform(pcl::getTransformation(-5.0f, 8.4f, 0.0f, 0.0f, 0.0f, -41.9872 * M_PI / 180.0));
      box_filter.filter(indices2);
      box_filter.filter(*calib_cloud);
    }
    else {
      pcl::copyPointCloud(*passThroughBetter, *calib_cloud);
    }
    break;
  }
  std::cout << "====================" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = mapping_vis(display_cloud);
  viewer->addLine(pcl::PointXYZ(0,18.35,0), pcl::PointXYZ(50,-24.15,0), 255, 0, 0, std::string("median_divider_line"), 0);
  viewer->addLine(pcl::PointXYZ(-5,18.75,0), pcl::PointXYZ(50,-29,0), 255, 0, 0, std::string("lane_2_3_divider_line"), 0);
  viewer->addLine(pcl::PointXYZ(-7,16.25,0), pcl::PointXYZ(50,-34,0), 255, 0, 0, std::string("lane_1_2_divider_line"), 0);
  viewer->addLine(pcl::PointXYZ(-9,14,0), pcl::PointXYZ(50,-39,0), 255, 0, 0, std::string("road_boundary_line"), 0);
  viewer->addLine(pcl::PointXYZ(3.9,-10,0), pcl::PointXYZ(20.9,10,0), 0, 255, 0, std::string("far_cut_off_line"), 0);
  viewer->addLine(pcl::PointXYZ(1.4025,20,0), pcl::PointXYZ(-15.5975,0,0), 0, 255, 0, std::string("close_cut_off_line"), 0);
  // viewer->addLine(pcl::PointXYZ(6.0,-2.97,0), pcl::PointXYZ(25.0,25.0,0), 0, 0, 255, std::string("test_line"), 0);
  for (const boost::filesystem::path & filename : dir_files) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughIn (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr bgBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusterBetter (new pcl::PointCloud<pcl::PointXYZI>);


    if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename.string(), *cloud) == -1) {
      PCL_ERROR ("Couldn't read pcd file \n");
      return (-1);
    }
    std::cout << filename.string() << std::endl;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *passThroughIn, indices); // this shouldn't affect anything but doing it anyway as it will make it easier to process data later

    if (pass_through_filtering){
      pcl::PassThrough<pcl::PointXYZI> x_filter; 
      x_filter.setInputCloud(passThroughIn);
      x_filter.setFilterFieldName("x");
      x_filter.setFilterLimits(filter_x_min, filter_x_max);
      x_filter.filter(*passThroughBetter);

      passThroughIn->clear();

      pcl::PassThrough<pcl::PointXYZI> z_filter;
      z_filter.setInputCloud(passThroughBetter);
      z_filter.setFilterFieldName("z");
      z_filter.setFilterLimits(filter_z_min, filter_z_max);
      z_filter.filter(*passThroughIn);

      passThroughBetter->clear();

      pcl::PassThrough<pcl::PointXYZI> y_filter;
      z_filter.setInputCloud(passThroughIn);
      z_filter.setFilterFieldName("y");
      z_filter.setFilterLimits(filter_y_min, filter_y_max);
      z_filter.filter(*passThroughBetter);
    }
    else {
      pcl::copyPointCloud(*passThroughIn, *passThroughBetter);
    }

    if (box_filtering){
      pcl::CropBox<pcl::PointXYZI> box_filter;
      std::vector<int> indices2;
      box_filter.setInputCloud(passThroughBetter);
      Eigen::Vector4f min_pt (0.0f, 0.0f, -20.0f, 1);
      Eigen::Vector4f max_pt (7.7f, 21.341f, 20.0f, 1);
      box_filter.setMin (min_pt);
      box_filter.setMax (max_pt);
      box_filter.setTransform(pcl::getTransformation(-5.0f, 8.4f, 0.0f, 0.0f, 0.0f, -41.9872 * M_PI / 180.0));
      box_filter.filter(indices2);
      box_filter.filter(*boxBetter);
    }
    else {
      pcl::copyPointCloud(*passThroughBetter, *boxBetter);
    }
    

    if (bg_filtering) {
      pcl::SegmentDifferences<pcl::PointXYZI> difference_segmenter;
      difference_segmenter.setInputCloud(boxBetter);
      difference_segmenter.setTargetCloud(calib_cloud);
      difference_segmenter.setDistanceThreshold(0.5);
      difference_segmenter.segment(*bgBetter);  
    } 
    else {
      pcl::copyPointCloud(*boxBetter, *bgBetter);
    }
    std::cout << "Points in frame: " << bgBetter->size() << std::endl;
    // // Clustering test

    if (bgBetter->size() > 0){
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr bgBetter_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
      pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZI>);
      pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
      pcl::copyPointCloud(*bgBetter, *bgBetter_with_normals);
      pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
      ne.setInputCloud(bgBetter);
      ne.setSearchMethod(search_tree);
      ne.setRadiusSearch(1.0);
      ne.compute(*bgBetter_with_normals);
      pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec (true);
      cec.setInputCloud(bgBetter_with_normals);
      cec.setConditionFunction(&customRegionGrowing);
      cec.setClusterTolerance(1.0);
      cec.setMinClusterSize(50);
      cec.segment(*clusters);

      for (int i = 0; i < clusters->size (); ++i)
      {
        int label = rand () % 8;
        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
          (*bgBetter)[(*clusters)[i].indices[j]].intensity = label;
      }
      std::cout << clusters->size() << " vehicles in frame" << std::endl;
      std::cout << "==================" << std::endl;
    } else {
      std::cout << "0 vehicles in frame" << std::endl;
    }
    
    viewer->updatePointCloud<pcl::PointXYZI>(bgBetter, "sample cloud");
    viewer->spinOnce(50);
  }
  return (0);
}
