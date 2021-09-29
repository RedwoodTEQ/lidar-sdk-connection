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

#include "tinyxml2/tinyxml2.h"

using namespace std::literals::chrono_literals;

# define M_PI           3.14159265358979323846  /* pi */

struct membuf : std::streambuf
{
    membuf(char* begin, char* end) {
        this->setg(begin, begin, end);
    }
};

typedef struct _camera_params {
  double pos_x;
  double pos_y;
  double pos_z;
  double view_x;
  double view_y;
  double view_z;
  double up_x;
  double up_y;
  double up_z;
} CameraParams, *CameraParamsPtr;

typedef struct _pass_through_filter_params {
  bool enabled;
  double filter_x_min;
  double filter_x_max;
  double filter_z_min;
  double filter_z_max;
  double filter_y_min;
  double filter_y_max;
} PassThroughFilterParams, *PassThroughFilterParamsPtr;

typedef struct _parameter_configuration {
  PassThroughFilterParams   ptfp;
} ParameterConfiguration, *ParameterConfigurationPtr;

pcl::visualization::PCLVisualizer::Ptr mapping_vis (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, CameraParams icp)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(icp.pos_x, icp.pos_y, icp.pos_z, icp.view_x, icp.view_y, icp.view_z, icp.up_x, icp.up_y, icp.up_z, 0);
    viewer->addLine(pcl::PointXYZ(0,18.35,0), pcl::PointXYZ(50,-24.15,0), 255, 0, 0, std::string("median_divider_line"), 0);
    viewer->addLine(pcl::PointXYZ(-5,18.75,0), pcl::PointXYZ(50,-29,0), 255, 0, 0, std::string("lane_2_3_divider_line"), 0);
    viewer->addLine(pcl::PointXYZ(-7,16.25,0), pcl::PointXYZ(50,-34,0), 255, 0, 0, std::string("lane_1_2_divider_line"), 0);
    viewer->addLine(pcl::PointXYZ(-9,14,0), pcl::PointXYZ(50,-39,0), 255, 0, 0, std::string("road_boundary_line"), 0);
    viewer->addLine(pcl::PointXYZ(3.9,-10,0), pcl::PointXYZ(20.9,10,0), 0, 255, 0, std::string("far_cut_off_line"), 0);
    viewer->addLine(pcl::PointXYZ(1.4025,20,0), pcl::PointXYZ(-15.5975,0,0), 0, 255, 0, std::string("close_cut_off_line"), 0);
    return (viewer);
}

bool customRegionGrowing(const pcl::PointXYZINormal& a, const pcl::PointXYZINormal& b, float squared_d) {
  return true;
}

void inputAndFilter(bool calibration, const char* input_filename, pcl::PointCloud<pcl::PointXYZI>::Ptr target, pcl::PointCloud<pcl::PointXYZI>::Ptr displayCloud, pcl::visualization::PCLVisualizer::Ptr v, ParameterConfiguration paracon) {
  bool bg_filtering = true;
  bool box_filtering = true;
  bool enable_clustering = true;

  struct archive *a;
  struct archive_entry *a_entry;
  int r;
  a = archive_read_new();
  archive_read_support_filter_gzip(a);
  archive_read_support_format_tar(a);
  r = archive_read_open_filename(a, input_filename, 1036288);
  if (r != ARCHIVE_OK){
    std::cerr << "archive open error occured!" << std::endl;
    return;
  }
  while (archive_read_next_header(a, &a_entry) == ARCHIVE_OK) {
    pcl::PCLPointCloud2::Ptr rawIn (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughIn (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxBetter (new pcl::PointCloud<pcl::PointXYZI>);

    std::cout << "Reading: " << archive_entry_pathname(a_entry) << std::endl;

    const auto fsize = archive_entry_size(a_entry);
    auto buffer = new char[fsize];
    auto read_result = archive_read_data(a, buffer, fsize);

    // Ensure number of bytes read from the current pcd file in archive matches its actual size
    if (read_result != fsize) {
      std::cerr << "archive read error occured!" << std::endl;
      break;
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
    // Skip 11 lines of header - this is required because readHeader appears to seek 1 line too many and will miss the data point
    for (auto i = 0; i < 11; i++) {
      std::string dontcare;
      std::getline(dataStream2, dontcare);
    }
    std::cout << "calibration: " << calibration << std::endl;
    pcd_reader.readBodyASCII(dataStream2, *rawIn, pcd_version);
    pcl::fromPCLPointCloud2(*rawIn, *cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *passThroughIn, indices); // this shouldn't affect anything but doing it anyway as it will make it easier to process data later

    if (paracon.ptfp.enabled){
      pcl::PassThrough<pcl::PointXYZI> x_filter; 
      x_filter.setInputCloud(passThroughIn);
      x_filter.setFilterFieldName("x");
      x_filter.setFilterLimits(paracon.ptfp.filter_x_min, paracon.ptfp.filter_x_max);
      x_filter.filter(*passThroughBetter);

      passThroughIn->clear();

      pcl::PassThrough<pcl::PointXYZI> z_filter;
      z_filter.setInputCloud(passThroughBetter);
      z_filter.setFilterFieldName("z");
      z_filter.setFilterLimits(paracon.ptfp.filter_z_min, paracon.ptfp.filter_z_max);
      z_filter.filter(*passThroughIn);

      passThroughBetter->clear();

      pcl::PassThrough<pcl::PointXYZI> y_filter;
      z_filter.setInputCloud(passThroughIn);
      z_filter.setFilterFieldName("y");
      z_filter.setFilterLimits(paracon.ptfp.filter_y_min, paracon.ptfp.filter_y_max);
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
      if (calibration) {
        box_filter.filter(*target);
      }
      else {
        box_filter.filter(*boxBetter);
      }
      
    }
    else {
      if (calibration) {
        pcl::copyPointCloud(*passThroughBetter, *target);
        return;
      }
    }
    if (bg_filtering) {
      pcl::SegmentDifferences<pcl::PointXYZI> difference_segmenter;
      difference_segmenter.setInputCloud(boxBetter);
      difference_segmenter.setTargetCloud(target);
      difference_segmenter.setDistanceThreshold(0.5);
      difference_segmenter.segment(*displayCloud);  
    } 
    else {
      pcl::copyPointCloud(*boxBetter, *displayCloud);
    }
    std::cout << "Points in frame: " << displayCloud->size() << std::endl;
    // // Clustering test

    if (displayCloud->size() > 0 && enable_clustering){
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr displayCloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
      pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZI>);
      pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
      pcl::copyPointCloud(*displayCloud, *displayCloud_with_normals);
      pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
      ne.setInputCloud(displayCloud);
      ne.setSearchMethod(search_tree);
      ne.setRadiusSearch(1.0);
      ne.compute(*displayCloud_with_normals);
      pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec (true);
      cec.setInputCloud(displayCloud_with_normals);
      cec.setConditionFunction(&customRegionGrowing);
      cec.setClusterTolerance(1.0);
      cec.setMinClusterSize(50);
      cec.segment(*clusters);

      for (int i = 0; i < clusters->size (); ++i)
      {
      int label = rand () % 8;
      for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
      (*displayCloud)[(*clusters)[i].indices[j]].intensity = label;
      }
      std::cout << clusters->size() << " vehicles in frame" << std::endl;
      std::cout << "==================" << std::endl;
    } else {
    std::cout << "0 vehicles in frame" << std::endl;
    }
    v->updatePointCloud<pcl::PointXYZI>(displayCloud, "sample cloud");
    v->spinOnce(50);
  }
}

int main () {
  // Load config
  tinyxml2::XMLDocument doc;
  doc.LoadFile("../lidarconfig.xml");
  if (doc.ErrorID() != 0) {
    std::cout << doc.ErrorID() << std::endl;
    std::cerr << "Could not load configuration file lidarconfig.xml" << std::endl;
    return 1;
  }
  std::cout << "Successfully loaded 'lidarconfig.xml'" << std::endl;
  ParameterConfiguration paracon = {};
  CameraParams icp = {};
  PassThroughFilterParams ptfp = {};

  // Read in initial camera params
  tinyxml2::XMLElement* icpElement = doc.FirstChildElement()->FirstChildElement("initialCameraParams");
  icpElement->FirstChildElement("pos_x")->QueryDoubleText(&icp.pos_x);
  icpElement->FirstChildElement("pos_y")->QueryDoubleText(&icp.pos_y);
  icpElement->FirstChildElement("pos_z")->QueryDoubleText(&icp.pos_z);
  icpElement->FirstChildElement("view_x")->QueryDoubleText(&icp.view_x);
  icpElement->FirstChildElement("view_y")->QueryDoubleText(&icp.view_y);
  icpElement->FirstChildElement("view_z")->QueryDoubleText(&icp.view_z);
  icpElement->FirstChildElement("up_x")->QueryDoubleText(&icp.up_x);
  icpElement->FirstChildElement("up_y")->QueryDoubleText(&icp.up_y);
  icpElement->FirstChildElement("up_z")->QueryDoubleText(&icp.up_z);

  // Read in pass through filter params
  tinyxml2::XMLElement* ptfpElement = doc.FirstChildElement()->FirstChildElement("passThroughFilter");
  ptfpElement->FirstChildElement("enabled")->QueryBoolText(&ptfp.enabled);
  ptfpElement->FirstChildElement("filter_x_min")->QueryDoubleText(&ptfp.filter_x_min);
  ptfpElement->FirstChildElement("filter_x_max")->QueryDoubleText(&ptfp.filter_x_max);
  ptfpElement->FirstChildElement("filter_y_min")->QueryDoubleText(&ptfp.filter_y_min);
  ptfpElement->FirstChildElement("filter_y_max")->QueryDoubleText(&ptfp.filter_y_max);
  ptfpElement->FirstChildElement("filter_z_min")->QueryDoubleText(&ptfp.filter_z_min);
  ptfpElement->FirstChildElement("filter_z_max")->QueryDoubleText(&ptfp.filter_z_max);

  paracon.ptfp = ptfp;

  pcl::PointCloud<pcl::PointXYZI>::Ptr calib_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr display_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = mapping_vis(display_cloud, icp);
  inputAndFilter(true, "westmead_pcd/calibration/lidar1.calibration.pcd.gz", calib_cloud, display_cloud, viewer, paracon);
  inputAndFilter(false, "westmead_pcd/lidar1.pcd.gz", calib_cloud, display_cloud, viewer, paracon);
  return 0;
}
