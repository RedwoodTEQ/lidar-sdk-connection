#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <istream>
#include <numeric>
#include <random>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/interprocess/streams/bufferstream.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <archive.h> 
#include <archive_entry.h>

#include "tinyxml2/tinyxml2.h"

using namespace std::literals::chrono_literals;

# define M_PI           3.14159265358979323846  /* pi */
# define MAX_FRAMES_DISAPPEARED 0              /* max number of frames an object can be disappeared for */

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

typedef struct _line_params {
  std::string name;
  double x1;
  double y1;
  double z1;
  double x2;
  double y2;
  double z2;
  double r;
  double g;
  double b;
} LineParams, *LineParamsPtr;

typedef struct _pass_through_filter_params {
  bool enabled;
  double filter_x_min;
  double filter_x_max;
  double filter_z_min;
  double filter_z_max;
  double filter_y_min;
  double filter_y_max;
} PassThroughFilterParams, *PassThroughFilterParamsPtr;

typedef struct _box_filter_params {
  bool enabled;
  float min_x;
  float min_y;
  float min_z;
  float max_x;
  float max_y;
  float max_z;
  float transform_x;
  float transform_y;
  float transform_z;
  float transform_roll;
  float transform_pitch;
  float transform_yaw;
} BoxFilterParams, *BoxFilterParamsPtr;

typedef struct _background_filter_params {
  bool enabled;
  double threshold;
} BackgroundFilterParams, *BackgroundFilterParamsPtr;


typedef struct _parameter_configuration {
  PassThroughFilterParams   ptfp;
  BoxFilterParams boxfp;
  BackgroundFilterParams bgfp;
  bool ec;
} ParameterConfiguration, *ParameterConfigurationPtr;

typedef struct _bounding_box {
  pcl::PointXYZ centroid;
  pcl::PointXYZ closeLeft;
  pcl::PointXYZ closeRight;
  pcl::PointXYZ farRight;
  double width;
  double length;
  double height;
} BoundingBox, *BoundingBoxPtr;

// https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
template <typename T>
std::vector<int> sort_indexes(const std::vector<T> &v) {
  std::vector<int> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(idx.begin(), idx.end(),
       [&v](double i1, double i2) {return v[i1] < v[i2];});
  return idx;
}

double distanceToBottom(pcl::PointXYZ p) {
  // Bottom plane: 200.5 * x - 10.1 * y - 144.4 * z - 2698.25 = 0
  double res = 200.5 * p.x - 10.1 * p.y - 144.4 * p.z + 2698.25;
  double dist = std::abs(res)/std::sqrt(std::pow(200.5,2) + std::pow(-10.1,2) + std::pow(-144.4, 2));
  return dist;
}

double distanceToTop(pcl::PointXYZ p) {
  // Top plane: -252.5 * x - 59.675 * y + 106.95 * z + 1457.45625 = 0
  double res = -252.5 * p.x - 59.675 * p.y + 106.95 * p.z + 1457.456;
  double dist = std::abs(res)/std::sqrt(std::pow(-252.5,2) + std::pow(-59.675,2) + std::pow(106.95, 2));
  return dist;
}

BoundingBox determineBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud, pcl::PointXYZ centroid) {
  BoundingBox bounding_box = {};
  pcl::PointXYZ closeLeftRefPoint = pcl::PointXYZ(10.75, -3.75, -3.5);
  pcl::PointXYZ closeRightRefPoint = pcl::PointXYZ(3, -10, -4.75);
  pcl::PointXYZ farRightRefPoint = pcl::PointXYZ(8.4, -23, 5);
  pcl::PointXYZ closeLeft;
  pcl::PointXYZ closeRight;
  pcl::PointXYZ farRight;
  pcl::PointXYZ topPoint;
  pcl::PointXYZ bottomPoint;
  auto min_dist_from_top = 100.0;
  auto min_dist_from_bottom = 100.0;
  auto min_dist = 1000.0;
  auto min_dist2 = 1000.0;
  auto min_dist3 = 1000.0;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it = cluster_cloud->begin(); it != cluster_cloud->end(); ++it ) {
    auto current = pcl::PointXYZ(it->x, it->y, it->z);
    auto dist = pcl::euclideanDistance(closeLeftRefPoint, current);
    auto dist2 = pcl::euclideanDistance(closeRightRefPoint, current);
    auto dist3 = pcl::euclideanDistance(farRightRefPoint, current);
    if (dist < min_dist) {
      min_dist = dist;
      closeLeft = current;
    }
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      closeRight = current;
    }
    if (dist3 < min_dist3) {
      min_dist3 = dist3;
      farRight = current;
    }
    auto dist_from_top = distanceToTop(current);
    auto dist_from_bottom = distanceToBottom(current);

    if (dist_from_top < min_dist_from_top) {
      min_dist_from_top = dist_from_top;
      topPoint = current;
    }
    if (dist_from_bottom < min_dist_from_bottom) {
      min_dist_from_bottom = min_dist_from_bottom;
      bottomPoint = current;
    }
    

  }
  auto vehicle_width = sqrt(pow((closeRight.x - closeLeft.x),2) + pow((closeRight.y - closeLeft.y),2));
  auto vehicle_length = sqrt(pow((farRight.x - closeRight.x),2) + pow((farRight.y - closeRight.y),2));
  bounding_box.centroid = centroid;
  bounding_box.closeLeft = closeLeft;
  bounding_box.closeRight = closeRight;
  bounding_box.farRight = farRight;
  bounding_box.width = vehicle_width;
  bounding_box.length = vehicle_length;
  bounding_box.height = pcl::euclideanDistance(topPoint, bottomPoint);
  return bounding_box;
}

pcl::PointXYZ calculateCentroid(pcl::PointCloud<pcl::PointXYZI>::Ptr input) {
  pcl::CentroidPoint<pcl::PointXYZI> centroid;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it = input->begin(); it != input->end(); ++it ) {
    centroid.add(*it);
  }
  pcl::PointXYZ c1;
  centroid.get(c1);
  return c1;
}

bool pointsEqual(pcl::PointXYZ a, pcl::PointXYZ b) {
  return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}

BoundingBox getBoundingBoxByCentroid(pcl::PointXYZ centroid, std::vector<BoundingBox> bbs) {
  for (auto it = bbs.begin(); it != bbs.end(); ++it) {
    if (pointsEqual(it->centroid, centroid)) {
      return *it;
    }
  }
}

std::pair<std::vector<double>, double> vehicleDimsAndSpeed(std::vector<BoundingBox> bbs) {
  
  auto max_width = 0.0;
  auto max_length = 0.0;
  auto max_height = 0.0;

  pcl::PointXYZ prev_centroid;
  int frame = 0;
  std::vector<double> centroid_spacing;
  for (auto it = bbs.begin(); it != bbs.end(); ++it) {
    if (it->width > max_width) {
      max_width = it->width;
    }
    if (it->length > max_length) {
      max_length = it->length;
    }
    if (it->height > max_height) {
      max_height = it->height;
    }
    if (frame > 0) {
      centroid_spacing.push_back(pcl::euclideanDistance(prev_centroid, it->centroid));
    }
    prev_centroid = it->centroid;
    frame++;
  }

  auto dims = std::vector<double>({max_width, max_length, max_height});
  auto avg_speed = 0.0;
  if (centroid_spacing.size() > 0) {
    avg_speed = std::accumulate(centroid_spacing.begin(), centroid_spacing.end(), 0.0) / centroid_spacing.size();
  }
  return std::pair<std::vector<double>,double>(dims, avg_speed);
}

void printVehicleInfo(int objectID, std::map<int, std::vector<BoundingBox>> vehicleInfo) {
  auto info = vehicleInfo[objectID];
  auto agg_info = vehicleDimsAndSpeed(info);
  auto dims = agg_info.first;
  std::cout << "==================" << std::endl;
  std::cout << "Summary of object ID " << objectID << ":" << std::endl;
  std::cout << "Visible for " << info.size() << " frames" << std::endl;
  if (dims.size() == 3) {
    std::cout << "L: " << dims[1] << " W: " << dims[0] << " H: " << dims[2] << std::endl;
  }
  auto speed_kms = std::round(agg_info.second * 36);
  std::cout << "Average speed: " << speed_kms << "km/h" << std::endl;
  std::cout << "==================" << std::endl;
}


// Object tracker derived from: https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/
void register_new (pcl::PointXYZ centroid, std::map<int, pcl::PointXYZ> &objects, std::map<int, int> &disappeared,  int &nextObjectID ) {
  objects[nextObjectID] = centroid;
  disappeared[nextObjectID] = 0;
  nextObjectID++;
}

void update(std::vector<pcl::PointXYZ> inputCentroids, std::map<int, pcl::PointXYZ> &objects, std::map<int, int> &disappeared, int &nextObjectID, int& totalCount, std::map<int, std::vector<BoundingBox>>& vehicleInfo ) {

  if (inputCentroids.size() == 0) {
    for (auto it = disappeared.begin(); it != disappeared.end(); ++it) {
      it->second++;
    }
    for (auto it = disappeared.cbegin(); it != disappeared.cend(); ) {
      if (it->second > MAX_FRAMES_DISAPPEARED) {
        printVehicleInfo(it->first, vehicleInfo);
        objects.erase(it->first);
        disappeared.erase(it++);
        vehicleInfo.erase(it->first);
      } else {
        ++it;
      }
    }
    return;
  }

  // If we are not currently tracking any objects
  if (objects.size() == 0) {
    for (auto it = inputCentroids.begin(); it != inputCentroids.end(); ++it) {
      register_new(*it, objects, disappeared, nextObjectID);
    }
  }
  else {
    // Grab the set of object IDs and corresponding centroids
    std::vector<int> objectIDs;
    std::vector<pcl::PointXYZ> objectCentroids;
    for (auto it = objects.begin(); it != objects.end(); ++it) {
      objectIDs.push_back(it->first);
      objectCentroids.push_back(it->second);
    }

    // Compute distance between each pair of object centroids and input centroids, respectively
    std::vector<std::vector<double>> D;
    for (auto it_oc = objectCentroids.begin(); it_oc != objectCentroids.end(); ++it_oc) {
      std::vector<double> matrix_row;
      for (auto it_ic = inputCentroids.begin(); it_ic != inputCentroids.end(); ++it_ic) {
        matrix_row.push_back(pcl::euclideanDistance(*it_oc, *it_ic));
      }
      D.push_back(matrix_row);
    }

    /* Remove objects that should not be tracked anymore due to
       unreasonably large minimum distance to current objects */
    std::vector<int> illegal_objectIDs;
    int obj_idx = 0;
    for (auto it = D.begin(); it != D.end(); ++it) {
      double min_dist = *(std::min_element(std::begin(*it), std::end(*it)));
      if (min_dist > 10) {
        illegal_objectIDs.push_back(objectIDs[obj_idx]);
      }
      obj_idx++;
    }

    // Erase illegal objects
    for (auto it = illegal_objectIDs.begin(); it != illegal_objectIDs.end(); ++it) {
      printVehicleInfo(*it, vehicleInfo);
      objects.erase(*it);
      vehicleInfo.erase(*it);
    }

    // Recompute matrix D - without illegal objects
    D.clear();
    objectIDs.clear();
    objectCentroids.clear();
    for (auto it = objects.begin(); it != objects.end(); ++it) {
      objectIDs.push_back(it->first);
      objectCentroids.push_back(it->second);
    }

    // Compute distance between each pair of object centroids and input centroids, respectively
    for (auto it_oc = objectCentroids.begin(); it_oc != objectCentroids.end(); ++it_oc) {
      std::vector<double> matrix_row;
      for (auto it_ic = inputCentroids.begin(); it_ic != inputCentroids.end(); ++it_ic) {
        matrix_row.push_back(pcl::euclideanDistance(*it_oc, *it_ic));
      }
      D.push_back(matrix_row);
    }

    /* Find the smallest value in each row and sort the row indexes 
      based on their minimum values so that the row with the smallest value
      is at the front of the index list
    */
    std::vector<double> rows_tmp;
    std::vector<int> cols_tmp;
    for (auto d_it = D.begin(); d_it != D.end(); ++d_it) {
      rows_tmp.push_back(*std::min_element(std::begin(*d_it), std::end(*d_it)));
      cols_tmp.push_back(std::distance(std::begin(*d_it), std::min_element(std::begin(*d_it), std::end(*d_it))));
    }

    std::vector<int> rows = sort_indexes(rows_tmp);
    std::vector<double> cols(cols_tmp.size());
    int i = 0;
    for (auto it = rows.begin(); it != rows.end(); ++it) {
      cols[*it] = cols_tmp[i];
      i++;
    }

    std::set<int> usedRows;
    std::set<int> usedCols;

    for (auto j = 0; j < rows.size(); j++) {
      auto row = rows[j];
      auto col = cols[j];
      if ((usedRows.find(row) != usedRows.end()) || (usedCols.find(col) != usedCols.end())) {
        continue;
      }
      auto objectID = objectIDs[row];
      objects[objectID] = inputCentroids[col];
      disappeared[objectID] = 0;
      usedRows.insert(row);
      usedCols.insert(col);
    }
    std::set<int> unusedRows;
    std::set<int> unusedCols;
    for (auto k = 0; k < D.size(); k++) {
      if (usedRows.find(k) == usedRows.end()) {
        unusedRows.insert(k);
      }
    }
    for (auto k = 0; k < D[0].size(); k++) {
      if (usedCols.find(k) == usedCols.end()) {
        unusedCols.insert(k);
      }
    }
    if (D.size() >= D[0].size()) {
      for (auto it = unusedRows.begin(); it != unusedRows.end(); ++it) {
        auto objectID = objectIDs[*it];
        disappeared[objectID]++;
      }
      for (auto it = disappeared.cbegin(); it != disappeared.cend(); ) {
        if (it->second > MAX_FRAMES_DISAPPEARED) {
          printVehicleInfo(it->first, vehicleInfo);
          objects.erase(it->first);
          disappeared.erase(it++);
          vehicleInfo.erase(it->first);
        } else {
          ++it;
        }
      }
    }
    else {
      for (auto it = unusedCols.begin(); it != unusedCols.end(); ++it) {
        register_new(inputCentroids[*it], objects, disappeared, nextObjectID);
        //std::cout << "REGISTER NEW SECOND" << std::endl;
      }
    }
  }

  return;
}



pcl::visualization::PCLVisualizer::Ptr mapping_vis (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, CameraParams icp, std::vector<LineParams>* lps)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(icp.pos_x, icp.pos_y, icp.pos_z, icp.view_x, icp.view_y, icp.view_z, icp.up_x, icp.up_y, icp.up_z, 0);
    for (auto it = lps->begin(); it != lps->end(); ++it) {
      viewer->addLine(pcl::PointXYZ(it->x1, it->y1, it->z1), pcl::PointXYZ(it->x2, it->y2, it->z2), it->r, it->g, it->b, it->name);
    }
    return (viewer);
}

// Always return true - just use distance thresholding
bool customRegionGrowing(const pcl::PointXYZINormal& a, const pcl::PointXYZINormal& b, float squared_d) {
  return true;
}

void inputAndFilter(bool calibration, const char* input_filename, pcl::PointCloud<pcl::PointXYZI>::Ptr target, pcl::PointCloud<pcl::PointXYZI>::Ptr displayCloud, pcl::visualization::PCLVisualizer::Ptr v, ParameterConfiguration paracon, bool exitAfterLastFrame) {

  // Set up centroid tracking
  auto nextObjectID = 0;
  auto totalCount = 0;
  auto lastCount = 0;
  std::map<int, pcl::PointXYZ> objects;
  std::map<int, int> disappeared;

  std::map<int, std::vector<BoundingBox>> vehicleInfo; // main data structure

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
  bool skip_files = false;
  while (archive_read_next_header(a, &a_entry) == ARCHIVE_OK) {
    pcl::PCLPointCloud2::Ptr rawIn (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughIn (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passThroughBetter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxBetter (new pcl::PointCloud<pcl::PointXYZI>);

    //std::cout << "Reading: " << archive_entry_pathname(a_entry) << std::endl;
    if (std::string("1634794377.778308868.ascii.pcd").compare(archive_entry_pathname(a_entry)) == 0) {
      skip_files = false;
    }
    if (skip_files && !calibration) {
      archive_read_data_skip(a);
      continue;
    }

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
    // Skip 11 lines of header - this is required because readHeader appears to seek 1 line too many and will miss the first data point
    for (auto i = 0; i < 11; i++) {
      std::string dontcare;
      std::getline(dataStream2, dontcare);
    }
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

    if (paracon.boxfp.enabled){
      pcl::CropBox<pcl::PointXYZI> box_filter;
      std::vector<int> indices2;
      box_filter.setInputCloud(passThroughBetter);
      Eigen::Vector4f min_pt (paracon.boxfp.min_x, paracon.boxfp.min_y, paracon.boxfp.min_z, 1);
      Eigen::Vector4f max_pt (paracon.boxfp.max_x, paracon.boxfp.max_y, paracon.boxfp.max_z, 1);
      box_filter.setMin (min_pt);
      box_filter.setMax (max_pt);
      box_filter.setTransform(pcl::getTransformation( paracon.boxfp.transform_x, 
                                                      paracon.boxfp.transform_y, 
                                                      paracon.boxfp.transform_z, 
                                                      paracon.boxfp.transform_roll, 
                                                      paracon.boxfp.transform_pitch, 
                                                      paracon.boxfp.transform_yaw ));
      box_filter.filter(indices2);
      // std::cout << indices2.size() << std::endl;
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
      else {
        pcl::copyPointCloud(*passThroughBetter, *boxBetter);
      }
    }
    if (paracon.bgfp.enabled) {
      pcl::SegmentDifferences<pcl::PointXYZI> difference_segmenter;
      difference_segmenter.setInputCloud(boxBetter);
      difference_segmenter.setTargetCloud(target);
      difference_segmenter.setDistanceThreshold(paracon.bgfp.threshold);
      difference_segmenter.segment(*displayCloud);  
    } 
    else {
      pcl::copyPointCloud(*boxBetter, *displayCloud);
    }
    
    //std::cout << "Points in frame: " << displayCloud->size() << std::endl;
    // // Clustering test

    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters); 
    std::vector<pcl::PointXYZ> centroids;
    std::vector<BoundingBox> bbs;
    if (paracon.ec) {
      if (displayCloud->size() > 0) {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr displayCloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZI>);
        
        pcl::copyPointCloud(*displayCloud, *displayCloud_with_normals);
        pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
        ne.setInputCloud(displayCloud);
        ne.setSearchMethod(search_tree);
        ne.setRadiusSearch(1.0);
        ne.compute(*displayCloud_with_normals);
        pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec (true);
        cec.setInputCloud(displayCloud_with_normals);
        cec.setConditionFunction(&customRegionGrowing);
        cec.setClusterTolerance(1.5);
        cec.setMinClusterSize(50);
        cec.segment(*clusters);

        for (int i = 0; i < clusters->size(); ++i) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::copyPointCloud(*displayCloud, (*clusters)[i].indices, *extracted_cloud);
          pcl::PointXYZ cent = calculateCentroid(extracted_cloud);
          BoundingBox bb = determineBoundingBox(extracted_cloud, cent);
          centroids.push_back(cent);
          bbs.push_back(bb);
        }
      }

      //std::cout << objects.size() << " tracked objects before update" << std::endl;
      update(centroids, objects, disappeared, nextObjectID, totalCount, vehicleInfo);
      if (lastCount != nextObjectID) {
        lastCount = nextObjectID;
        
        std::cout << "Total count: " << nextObjectID << std::endl;
        
      }
      for (auto it = objects.begin(); it != objects.end(); ++it) {
        auto obj_bb = getBoundingBoxByCentroid(objects[it->first], bbs);

        if (vehicleInfo.find(it->first) != vehicleInfo.end()) {
          vehicleInfo[it->first].push_back(obj_bb);
        }
        else {
          std::vector<BoundingBox> bb_vec;
          bb_vec.push_back(obj_bb);
          vehicleInfo.insert(std::pair<int, std::vector<BoundingBox>>(it->first, bb_vec));
        }
      }
      // for (auto it = objects.begin(); it != objects.end(); ++it) {
      //   auto obj_id = it->first;
      //   auto centroid = it->second;
      //   for (auto bbm_it = bbMap.begin(); bbm_it != bbMap.end(); ++bbm_it) {
      //     // if (pointsEqual(it->second, bbm_it->first)) {
      //     //   auto obj_bb = bbm_it->second;
      //     //   std::cout << "==================" << std::endl;
      //     //   std::cout << "Dimensions of object ID " << it->first << ":\n";
      //     //   std::cout << "L: " << obj_bb.length << " W: " << obj_bb.width << "H: " << obj_bb.height << std::endl;
      //     //   std::cout << "Vehicle class: " << "car" << std::endl;
      //     //   std::cout << "==================" << std::endl;
      //     // }
          
      //   }


      // }
      
      //std::cout << objects.size() << " tracked objects after update" << std::endl;
      //std::cout << clusters->size() << " total centroids in frame" << std::endl;
      
    }
    v->updatePointCloud<pcl::PointXYZI>(displayCloud, "sample cloud");
    if (exitAfterLastFrame) {
      v->spinOnce(50);
    }
    else {
      while (1) {
        v->spinOnce(50);
      }
    }
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
  BoxFilterParams boxfp = {};
  BackgroundFilterParams bgfp = {};
  auto lines = new std::vector<LineParams>();
  auto exitOnLastFrame = false;
  auto enableClustering = true;

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

  // Read in lines
  tinyxml2::XMLElement* drawLinesElement = doc.FirstChildElement()->FirstChildElement("drawLines");
  for(tinyxml2::XMLElement* e = drawLinesElement->FirstChildElement("line"); e != NULL; e = e->NextSiblingElement("line")) {
    LineParams lp = {};
    lp.name = e->FirstChildElement("name")->GetText();
    e->FirstChildElement("x1")->QueryDoubleText(&lp.x1);
    e->FirstChildElement("y1")->QueryDoubleText(&lp.y1);
    e->FirstChildElement("z1")->QueryDoubleText(&lp.z1);
    e->FirstChildElement("x2")->QueryDoubleText(&lp.x2);
    e->FirstChildElement("y2")->QueryDoubleText(&lp.y2);
    e->FirstChildElement("z2")->QueryDoubleText(&lp.z2);
    e->FirstChildElement("r")->QueryDoubleText(&lp.r);
    e->FirstChildElement("g")->QueryDoubleText(&lp.g);
    e->FirstChildElement("b")->QueryDoubleText(&lp.b);
    lines->push_back(lp);
  }

  // Decide if we should exit vtk on last frame (debug mode)
  tinyxml2::XMLElement* exitOnlastElement = doc.FirstChildElement()->FirstChildElement("exitAfterLastFrame");
  exitOnlastElement->QueryBoolText(&exitOnLastFrame);

  // Decide if we should enable clustering (default: on, debug: off)
  tinyxml2::XMLElement* enableClusteringElement = doc.FirstChildElement()->FirstChildElement("enableClustering");
  enableClusteringElement->QueryBoolText(&enableClustering);

  // Read in pass through filter params
  tinyxml2::XMLElement* ptfpElement = doc.FirstChildElement()->FirstChildElement("passThroughFilter");
  ptfpElement->FirstChildElement("enabled")->QueryBoolText(&ptfp.enabled);
  ptfpElement->FirstChildElement("filter_x_min")->QueryDoubleText(&ptfp.filter_x_min);
  ptfpElement->FirstChildElement("filter_x_max")->QueryDoubleText(&ptfp.filter_x_max);
  ptfpElement->FirstChildElement("filter_y_min")->QueryDoubleText(&ptfp.filter_y_min);
  ptfpElement->FirstChildElement("filter_y_max")->QueryDoubleText(&ptfp.filter_y_max);
  ptfpElement->FirstChildElement("filter_z_min")->QueryDoubleText(&ptfp.filter_z_min);
  ptfpElement->FirstChildElement("filter_z_max")->QueryDoubleText(&ptfp.filter_z_max);

  // Read in box filtering params
  tinyxml2::XMLElement* boxfpElement = doc.FirstChildElement()->FirstChildElement("boxFilter");
  boxfpElement->FirstChildElement("enabled")->QueryBoolText(&boxfp.enabled);
  tinyxml2::XMLElement* boxfpMinPointElement = boxfpElement->FirstChildElement("minPoint");
  boxfpMinPointElement->FirstChildElement("x")->QueryFloatText(&boxfp.min_x);
  boxfpMinPointElement->FirstChildElement("y")->QueryFloatText(&boxfp.min_y);
  boxfpMinPointElement->FirstChildElement("z")->QueryFloatText(&boxfp.min_z);
  tinyxml2::XMLElement* boxfpMaxPointElement = boxfpElement->FirstChildElement("maxPoint");
  boxfpMaxPointElement->FirstChildElement("x")->QueryFloatText(&boxfp.max_x);
  boxfpMaxPointElement->FirstChildElement("y")->QueryFloatText(&boxfp.max_y);
  boxfpMaxPointElement->FirstChildElement("z")->QueryFloatText(&boxfp.max_z);
  tinyxml2::XMLElement* boxfpTransformElement = boxfpElement->FirstChildElement("transformation");
  boxfpTransformElement->FirstChildElement("x")->QueryFloatText(&boxfp.transform_x);
  boxfpTransformElement->FirstChildElement("y")->QueryFloatText(&boxfp.transform_y);
  boxfpTransformElement->FirstChildElement("z")->QueryFloatText(&boxfp.transform_z);
  boxfpTransformElement->FirstChildElement("roll")->QueryFloatText(&boxfp.transform_roll);
  boxfpTransformElement->FirstChildElement("pitch")->QueryFloatText(&boxfp.transform_pitch);
  boxfpTransformElement->FirstChildElement("yaw")->QueryFloatText(&boxfp.transform_yaw);

  // Read in background filtering params
  tinyxml2::XMLElement* bgfpElement = doc.FirstChildElement()->FirstChildElement("backgroundFilter");
  bgfpElement->FirstChildElement("enabled")->QueryBoolText(&bgfp.enabled);
  bgfpElement->FirstChildElement("threshold")->QueryDoubleText(&bgfp.threshold);

  paracon.ptfp = ptfp;
  paracon.boxfp = boxfp;
  paracon.bgfp = bgfp;
  paracon.ec = enableClustering;

  pcl::PointCloud<pcl::PointXYZI>::Ptr calib_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr display_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = mapping_vis(display_cloud, icp, lines);
  inputAndFilter(true, "overpass/lidar1.calibration.ascii.pcd.gz", calib_cloud, display_cloud, viewer, paracon, exitOnLastFrame);
  inputAndFilter(false, "overpass/lidar1.ascii.pcd.gz", calib_cloud, display_cloud, viewer, paracon, exitOnLastFrame);
  //inputAndFilter(true, "overpass/lidar1.calibration.ascii.pcd.gz", calib_cloud, display_cloud, viewer, paracon);
  //inputAndFilter(false, "overpass/full.pcd.gz", calib_cloud, display_cloud, viewer, paracon);
  return 0;
}
