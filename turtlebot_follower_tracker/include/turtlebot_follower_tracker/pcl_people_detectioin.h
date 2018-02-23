#ifndef PCL_PEOPLE_DETECTIOIN_H
#define PCL_PEOPLE_DETECTIOIN_H

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/people/person_cluster.h>

#include <vector>
#include <set>

namespace turtlebot_guide
{

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef std::vector<pcl::people::PersonCluster<PointT> > P_Clusters;

class PCLPeopleDetection
{
public:

  PCLPeopleDetection(const double &camera_height);

  ~PCLPeopleDetection();

  void setInputCloud(const PointCloud &input_cloud);

  bool compute();

  Eigen::Affine3f getPeoplePose(){ return people_pose_; }

private:

  void initialize();

  Eigen::Hyperplane<float, 3> getGroundPlane();

  PointCloud::Ptr scene_cloud_;

  pcl::people::PersonClassifier<pcl::RGB> person_classifier_;
  std::string svm_filename_;
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector_;

  double camera_height_;
  double voxel_size_;
  double min_confidence_;

  Eigen::Affine3f people_pose_;

};
} // namespace turtlebot_guide


#endif // PCL_PEOPLE_DETECTIOIN_H
