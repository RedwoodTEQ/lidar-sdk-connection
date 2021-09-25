#include <algorithm>
#include <chrono>
#include <iostream>
#include <istream>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/interprocess/streams/bufferstream.hpp>
#include <boost/lexical_cast.hpp>

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

#include <archive.h> 
#include <archive_entry.h>

using namespace std::literals::chrono_literals;

# define M_PI           3.14159265358979323846  /* pi */

struct membuf : std::streambuf
{
    membuf(char* begin, char* end) {
        this->setg(begin, begin, end);
    }
};

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
  bool pass_through_filtering = true;
  bool box_filtering = true;
  bool enable_clustering = true;

  pcl::PointCloud<pcl::PointXYZI>::Ptr calib_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  struct archive *calib_a;
  struct archive_entry *calib_a_entry;
  int calib_r;
  calib_a = archive_read_new();
  archive_read_support_filter_gzip(calib_a);
  archive_read_support_format_tar(calib_a);
  calib_r = archive_read_open_filename(calib_a, "westmead_pcd/lidar1.calibration.pcd.gz", 1036288);
  if (calib_r != ARCHIVE_OK) {
    return 1;
  }
  while (archive_read_next_header(calib_a, &calib_a_entry) == ARCHIVE_OK) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughIn (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr bgBetter (new pcl::PointCloud<pcl::PointXYZI>);

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr display_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = mapping_vis(display_cloud);
  viewer->addLine(pcl::PointXYZ(0,18.35,0), pcl::PointXYZ(50,-24.15,0), 255, 0, 0, std::string("median_divider_line"), 0);
  viewer->addLine(pcl::PointXYZ(-5,18.75,0), pcl::PointXYZ(50,-29,0), 255, 0, 0, std::string("lane_2_3_divider_line"), 0);
  viewer->addLine(pcl::PointXYZ(-7,16.25,0), pcl::PointXYZ(50,-34,0), 255, 0, 0, std::string("lane_1_2_divider_line"), 0);
  viewer->addLine(pcl::PointXYZ(-9,14,0), pcl::PointXYZ(50,-39,0), 255, 0, 0, std::string("road_boundary_line"), 0);
  viewer->addLine(pcl::PointXYZ(3.9,-10,0), pcl::PointXYZ(20.9,10,0), 0, 255, 0, std::string("far_cut_off_line"), 0);
  viewer->addLine(pcl::PointXYZ(1.4025,20,0), pcl::PointXYZ(-15.5975,0,0), 0, 255, 0, std::string("close_cut_off_line"), 0);

  // Open test data archive
  // How to read file within archives: https://stackoverflow.com/questions/31500340/how-to-read-file-within-archives/31500763
  struct archive *a;
  struct archive_entry *a_entry;
  int r;
  // Support opening gzipped tar archive
  a = archive_read_new();
  archive_read_support_filter_gzip(a);
  archive_read_support_format_tar(a);
  r = archive_read_open_filename(a, "westmead_pcd/lidar1.pcd.gz", 1036288);
  if (r != ARCHIVE_OK){
    return 1;
  }
  while (archive_read_next_header(a, &a_entry) == ARCHIVE_OK) {
    pcl::PCLPointCloud2::Ptr rawIn (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughIn (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr bgBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusterBetter (new pcl::PointCloud<pcl::PointXYZI>);

    const auto fsize = archive_entry_size(a_entry);
    auto buffer = new char[fsize];
    auto read_result = archive_read_data(a, buffer, fsize);

    // Ensure number of bytes read from the current pcd file in archive matches its actual size
    if (read_result != fsize) {
      return 1;
    }
    // Convert char array to istream
    membuf sbuf(buffer, buffer + fsize);
    membuf sbuf2(buffer, buffer + fsize);
    std::istream dataStream(&sbuf);
    std::istream dataStream2(&sbuf2);
    pcl::PCDReader pcd_reader;
    auto test_eigenvector = Eigen::Vector4f();
    auto test_eigenquaternion = Eigen::Quaternionf::Identity();
    auto offset = 0u;
    auto data_type = 0;
    auto pcd_version = 0;
    pcd_reader.readHeader(dataStream, *rawIn, test_eigenvector, test_eigenquaternion, pcd_version, data_type, offset);
    for (auto i = 0; i < 11; i++) {
      std::string dontcare;
      std::getline(dataStream2, dontcare);
    }
    pcd_reader.readBodyASCII(dataStream2, *rawIn, pcd_version);
    pcl::fromPCLPointCloud2(*rawIn, *cloud);
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

    if (bgBetter->size() > 0 && enable_clustering){
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
  return 0;
}
