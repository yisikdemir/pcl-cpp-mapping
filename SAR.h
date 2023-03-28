// ------------------------------------------------------------------------------
//
// Author: Yunus Emre Işıkdemir
// 
// Create Date: 01/08/2021
//
// Project Name: Semantic Mapping and Plane Segmentation on Search and Rescue Arenas
//
// Description: This is the header file of the application
// 
// ------------------------------------------------------------------------------


// prevent multiple inclusions of header
#ifndef SAR_H
#define SAR_H

// Include ROS
#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

// libraries for plane segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// libraries for Region Growing
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

// Standard Libraries
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

// definitions
using namespace std;

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB PointType2;

typedef pcl::PointCloud<PointType> CloudType;
typedef pcl::PointCloud<PointType2> CloudType2;

// Updating the to_string() function
namespace patch

{
    template < typename T > string to_string( const T& n )
    {
        ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

// Comparator for the sets of the points
struct classcomp
{
   bool operator() (const PointType& p1, const PointType& p2) const
   {
      return p1.x < p2.x;
   }
};


// Search and Rescue Class
class SAR
{
  
public:
   // Constructor for SAR class.
   SAR(const string pcdPath, const int numPCD);   
   
   // Prints the number of points of each cloud
   void cloudMapPrinter(int numToPrint = 20);
  
   // Get pcds from a file from given path and stores the set of points format with removing duplicate points.
   map<int, set<PointType, classcomp> > getUniquePCDMap(const string pth, const int range);

   // Pointmap to cloudmap conversion.
   map<int, CloudType::Ptr > PointMap2CloudMap2(map<int, set<PointType, classcomp> > m);

   // Aggregrate all the pcds and visualize as a whole entire environment.
   void visualizeAllClouds();

   // Visualize a cloud including PointXYZ.
   void visualizeMyCloud(CloudType::Ptr output);

   // Visualize a cloud including PointXYZRGB.
   void visualizeMyRGBCloud(CloudType2::Ptr output);

   // Get the model output from txt located on given path.
   vector<vector<double> > getModelOutput(string path);

   // Write dummy labels to text for the model.
   void dummyTextWriter(int sizeofcld);

   // Colorize the cloud with respect to plane type.
   CloudType2::Ptr colorizePoints(string planeType, CloudType2::Ptr &cloud);

   // xyz assign to model output - rgb assign to zero
   map<string, CloudType2::Ptr> xyzAssigner(vector<vector<double> > v);

   // Extracting indices from a PointCloud using RANSAC.
   map<string, vector<CloudType2::Ptr> > extractIndicesRansac(map<string, CloudType2::Ptr> planeType);

   // Extract bounding boxes and visualize on given plane.
   void getBoundingBox(CloudType2::Ptr eachPlane);

   // Region Growing coefficients and inliers.
   void RegionGrowingFnc(string pcdName);

   // Rotation of cloud about three axis.
   void rotatethis(CloudType2::Ptr &source_cloud, int xAngle=0, int yAngle=0, int zAngle=0);

   // Set pointsmap data member.
   void setPointsmap(map<int, set<PointType, classcomp> > m);

   // Set cloudsmap data member.
   void setCloudsmap(map<int, CloudType::Ptr > s);

   // Get pointsmap data member.
   map<int, set<PointType, classcomp> > getPointsmap();

   // Get cloudsmap data member.
   map<int, CloudType::Ptr > getCloudsmap();

   // Destructor of SAR class.
   ~SAR();


private:
      // Path for loading pcd files.
   const string path;

   // How many pcd to read.
   const int numOfPCD;

   // PCD stored in the form of Map of SetofPoints with predicate.
   map<int, set<PointType, classcomp> > pointsMap;

   // PCD stored in the form of Map of PointClouds.
   map<int, CloudType::Ptr > cloudsMap;

};


#endif 



