// ------------------------------------------------------------------------------
//
// Author: Yunus Emre Işıkdemir
// 
// Create Date: 01/08/2021
//
// Project Name: Semantic Mapping and Plane Segmentation on Search and Rescue Arenas
//
// Description: Implementation of the application that is defined in header file.
// 
// ------------------------------------------------------------------------------

// Include Standard Libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <vector>
#include <set>
#include <iomanip>
#include <string>
#include <cmath>
#include <algorithm>
#include <map>
#include <numeric>
#include "SAR.h"

// definitions
using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB PointType2;
typedef pcl::PointCloud < PointType > CloudType;
typedef pcl::PointCloud < PointType2 > CloudType2;

// Class Definitions
SAR::SAR(const string pcdPath,
  const int numPCD): path(pcdPath), numOfPCD(numPCD) {
  setPointsmap(getUniquePCDMap(pcdPath, numPCD));
  setCloudsmap(PointMap2CloudMap2(pointsMap));
}

// Set pointsmap data member.
void SAR::setPointsmap(map < int, set < PointType, classcomp > > m) {
  pointsMap = m;
}

 // Set cloudsmap data member.
void SAR::setCloudsmap(map < int, CloudType::Ptr > m) {
  cloudsMap = m;
}

// Get pointsmap data member.
map < int, set < PointType, classcomp > > SAR::getPointsmap() {
  return pointsMap;
}

// Get cloudsmap data member.
map < int, CloudType::Ptr > SAR::getCloudsmap() {
  return cloudsMap;
}

// Prints the number of points of each cloud
void SAR::cloudMapPrinter(int numToPrint) {

  cout << "\n\n--Points--\n";

  for_each(cloudsMap.begin(), next(cloudsMap.begin(), numToPrint), [](const pair < int, CloudType::Ptr > & p) {
    cout << p.second -> points.size() << "points\n";
  });

  cout << "\n---Total points in whole env : " << accumulate(pointsMap.begin(), pointsMap.end(), 0,
    [](int cumulative,
      const pair < int, set < PointType, classcomp > > & p) {
      return cumulative + p.second.size();
    }) << "\n";
}

// Aggregrate all the pcds and visualize as a whole entire environment.
void SAR::visualizeAllClouds() {
  CloudType::Ptr output(new CloudType);

  for_each(cloudsMap.begin(), cloudsMap.end(), [output](const pair < int, CloudType::Ptr > & p) {
    * output += * (p.second);
  });

  visualizeMyCloud(output);
}

map < int, set < PointType, classcomp > > SAR::getUniquePCDMap(const string pth,
  const int range) {

  int i = 0;

  // String that stores the each file name
  string c;

  // Storage for clouds
  map < int, set < PointType, classcomp > > temp;
  for (int i = 1; i <= range; i++) {
    c = pth + patch::to_string(i) + ".pcd";

    CloudType::Ptr cloud1(new CloudType);

    if (pcl::io::loadPCDFile(c, * cloud1) != -1) {

      for (int j = 0; j < cloud1 -> points.size(); j++) {
        temp[i].insert(cloud1 -> points[j]);
      }
    }
  }
  return temp;
}

// Pointmap to cloudmap conversion.
map < int, CloudType::Ptr > SAR::PointMap2CloudMap2(map < int, set < PointType, classcomp > > m) {
  map < int, CloudType::Ptr > temp;

  int indexer = 0;
  for (int i = 1; i <= m.size(); i++) {
    CloudType::Ptr cld(new pcl::PointCloud < pcl::PointXYZ > );
    cld -> points.resize(m[i].size());

    for (set < PointType, classcomp > ::iterator it = m[i].begin(); it != m[i].end(); it++) {
      cld -> points[indexer].x = it -> x;
      cld -> points[indexer].y = it -> y;
      cld -> points[indexer].z = it -> z;
      indexer++;
    }

    cld -> height = cld -> points.size();
    cld -> width = 1;
    temp[i] = cld;
    indexer = 0;
  }

  return temp;
}

// Visualize a cloud including PointXYZ
void SAR::visualizeMyCloud(CloudType::Ptr output) {
  pcl::visualization::PCLVisualizer viewer;
  pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > color(output, 155, 236, 249);
  viewer.addPointCloud < pcl::PointXYZ > (output, color, "SAR_MAP");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "SAR_MAP");

  // Keep viewer running
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

// Visualize a cloud including PointXYZRGB.
void SAR::visualizeMyRGBCloud(CloudType2::Ptr denemex) {
  pcl::visualization::PCLVisualizer viewer;
  pcl::visualization::PointCloudColorHandlerRGBField < PointType2 > rgb(denemex);
  viewer.addPointCloud < PointType2 > (denemex, rgb, "clouds_deneme");

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

// Get the model output from txt located on given path.
vector < vector < double > > SAR::getModelOutput(string path) {
  int lines;

  ifstream infile(path.c_str());
  lines = count(std::istreambuf_iterator < char > (infile), istreambuf_iterator < char > (), '\n');
  infile.seekg(ios::beg);

  int indexer = 0;
  vector < vector < double > > vekt;
  vekt.resize(lines);

  // Modelin output text'i: x1:x x2:y x3:z x5:label hepsi aynı satırda boşluk ile ayrılmış
  double x1, x2, x3, x4, x5;
  while (infile >> x1 >> x2 >> x3 >> x4 >> x5) {
    vekt[indexer].resize(4);
    vekt[indexer][0] = x1;
    vekt[indexer][1] = x2;
    vekt[indexer][2] = x3;
    vekt[indexer][3] = x5;
    indexer++;
  }

  return vekt;
}

// Write dummy labels to text for the model.
void SAR::dummyTextWriter(int sizeofcld) {

  ofstream myfile("dgcnn_ramp_evaluate/sem_seg/robotTestDataset/labels_scene_1.txt");
  ofstream myfile2("dgcnn_ramp_evaluate/sem_seg/robotTestDataset/RGB_scene_1.txt");

  if (myfile.is_open() && myfile2.is_open())

  {

    for (int i = 0; i < sizeofcld; i++) {
      myfile << "3\n";
      myfile2 << "155\t155\t155\n";
    }

    myfile.close();
    myfile2.close();
  } else
    cout << "Unable to open file\n";
}

// Colorize the cloud with respect to plane type.
CloudType2::Ptr SAR::colorizePoints(string planeType, CloudType2::Ptr & cl) {
  for (int i = 0; i < cl -> points.size(); i++) {
    ( * cl).points[i].r = (planeType == "inclined") ? 0 : 255;
    ( * cl).points[i].g = (planeType == "ground") ? 255 : 0;
    ( * cl).points[i].b = (planeType == "inclined" || planeType == "flat") ? 255 : 0;
  }
}

map < string, CloudType2::Ptr > SAR::xyzAssigner(vector < vector < double > > v) {
  map < string, CloudType2::Ptr > planes;

  CloudType2::Ptr temp(new CloudType2);
  temp -> height = 1;
  temp -> width = v.size();
  temp -> points.resize(temp -> width * temp -> height);

  // w:wall  g:ground  i:inclined ramp  f:flat ramp
  CloudType2::Ptr wl(new CloudType2), gnd(new CloudType2), inc(new CloudType2), flt(new CloudType2);

  for (int i = 0; i < v.size(); i++) {
    ( * temp).points[i].x = v[i][0];
    ( * temp).points[i].y = v[i][1];
    ( * temp).points[i].z = v[i][2];
    ( * temp).points[i].r = (v[i][3] == 0) ? 0 : 255;
    ( * temp).points[i].g = (v[i][3] == 3) ? 255 : 0;
    ( * temp).points[i].b = (v[i][3] == 0 || v[i][3] == 2) ? 255 : 0;

    // Label değeri v[i][3] te tutuluyor. Buna göre bölgeleri ilgili cloud'a atalım
    (v[i][3] == 0) ? inc -> points.push_back(( * temp).points[i]):
      (v[i][3] == 1) ? wl -> points.push_back(( * temp).points[i]) :
      (v[i][3] == 2) ? flt -> points.push_back(( * temp).points[i]) :
      gnd -> points.push_back(( * temp).points[i]);
  }

  //pcl::io::savePCDFileASCII("rennkli.pcd", *clds);

  planes["inclined"] = inc;
  planes["wall"] = wl;
  planes["flat"] = flt;
  planes["ground"] = gnd;

  return planes;
}

 // Extracting indices from a PointCloud using RANSAC.
map < string, vector < CloudType2::Ptr > > SAR::extractIndicesRansac(map < string, CloudType2::Ptr > planeType) {

  map < string, vector < CloudType2::Ptr > > segmentedPlane;
  pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
  CloudType::Ptr cloud_filtered(new CloudType), cloud_p(new CloudType), cloud_f(new CloudType);

  for (map < string, CloudType2::Ptr > ::iterator itr = planeType.begin(); itr != planeType.end(); itr++) {

    pcl::toPCLPointCloud2( * itr -> second, * cloud_blob);
    
    pcl::VoxelGrid < pcl::PCLPointCloud2 > sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.01 f, 0.01 f, 0.01 f);
    sor.filter( * cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2( * cloud_filtered_blob, * cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation < pcl::PointXYZ > seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.02); //0.1

    // Create the filtering object
    pcl::ExtractIndices < pcl::PointXYZ > extract;

    int i = 0, nr_points = (int) cloud_filtered -> size();
    while (cloud_filtered -> size() > 0.1 * nr_points) //0.3
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment( * inliers, * coefficients);
      if (inliers -> indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the inliers
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter( * cloud_p);

      // RGB convert
      CloudType2::Ptr cloud_pRGB(new CloudType2);
      pcl::copyPointCloud( * cloud_p, * cloud_pRGB);
      colorizePoints(itr -> first, cloud_pRGB);

      if (cloud_pRGB -> points.size() > 100)
        segmentedPlane[itr -> first].push_back(cloud_pRGB);

      // Create the filtering object
      extract.setNegative(true);
      extract.filter( * cloud_f);
      cloud_filtered.swap(cloud_f);
      i++;
    }
  }

  return segmentedPlane;

}

// Extract bounding boxes and visualize on given plane.
void SAR::getBoundingBox(CloudType2::Ptr eachPlane) {
  PointType2 minPt, maxPt;
  pcl::getMinMax3D( * eachPlane, minPt, maxPt);

  cout << "Max x: " << maxPt.x << std::endl <<
    "Max y: " << maxPt.y << std::endl <<
    "Max z: " << maxPt.z << std::endl <<
    "Min x: " << minPt.x << std::endl <<
    "Min y: " << minPt.y << std::endl <<
    "Min z: " << minPt.z << std::endl;

  pcl::visualization::PCLVisualizer viewerr;
  pcl::visualization::PointCloudColorHandlerRGBField < PointType2 > rgb(eachPlane);
  viewerr.addPointCloud < PointType2 > (eachPlane, rgb, "clouds_denemes");
  viewerr.addCube(minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);

  // Keep viewer running
  while (!viewerr.wasStopped()) {
    viewerr.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

}

// Region Growing coefficients and inliers.
void SAR::RegionGrowingFnc(string pcdName) {
  CloudType::Ptr cloud(new CloudType);
  pcl::io::loadPCDFile(pcdName, * cloud);

  pcl::search::Search < pcl::PointXYZ > ::Ptr tree(new pcl::search::KdTree < pcl::PointXYZ > );
  pcl::PointCloud < pcl::Normal > ::Ptr normals(new pcl::PointCloud < pcl::Normal > );
  pcl::NormalEstimation < pcl::PointXYZ, pcl::Normal > normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);

  normal_estimator.setKSearch(50);
  normal_estimator.compute( * normals);

  pcl::IndicesPtr indices(new std::vector < int > );
  pcl::PassThrough < pcl::PointXYZ > pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter( * indices);

  pcl::RegionGrowing < pcl::PointXYZ, pcl::Normal > reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);

  std::vector < pcl::PointIndices > clusters;
  reg.extract(clusters);

  std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;

  while (counter < clusters[0].indices.size()) {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;

  pcl::PointCloud < pcl::PointXYZRGB > ::Ptr colored_cloud = reg.getColoredCloud();
  pcl::visualization::CloudViewer viewer("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped()) {}
}

// Rotation of cloud about three axis.
void SAR::rotatethis(CloudType2::Ptr & source_cloud, int xAngle, int yAngle, int zAngle) {

  Eigen::Affine3f transform(Eigen::Affine3f::Identity());

  transform.linear() = (Eigen::Matrix3f) Eigen::AngleAxisf(DEG2RAD(xAngle), Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(DEG2RAD(yAngle), Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(DEG2RAD(zAngle), Eigen::Vector3f::UnitZ());

  Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
  pcl::compute3DCentroid( * source_cloud, centroid);
  Eigen::Vector4f centroid_new(Eigen::Vector4f::Zero());
  centroid_new.head < 3 > () = transform.rotation() * centroid.head < 3 > ();
  transform.translation() = centroid.head < 3 > () - centroid_new.head < 3 > ();

  pcl::transformPointCloud( * source_cloud, * source_cloud, transform);
}

SAR::~SAR() {

}