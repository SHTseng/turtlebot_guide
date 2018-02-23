#include <turtlebot_follower_tracker/pcl_people_detectioin.h>

namespace turtlebot_guide
{

PCLPeopleDetection::PCLPeopleDetection(const double &camera_height):
  camera_height_(camera_height),
  voxel_size_(0.06),
  min_confidence_(-1.5)
{
  svm_filename_ = ros::package::getPath("turtlebot_follower_tracker")+"/params/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  ROS_INFO("start people detection initialize");
  initialize();
}

PCLPeopleDetection::~PCLPeopleDetection() {}

void PCLPeopleDetection::initialize()
{
  if(!person_classifier_.loadSVMFromFile(svm_filename_)){
    ROS_ERROR("people classifier SVM file loading fail");
    exit(1);
  }
  ROS_INFO("Loaded SVM file");

  people_detector_.setVoxelSize(voxel_size_);                        // set the voxel size
  people_detector_.setClassifier(person_classifier_);                // set person classifier
  people_detector_.setPersonClusterLimits(1.0, 2.5, 0.1, 8.0);
  people_detector_.setSamplingFactor(1);

  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0;
  people_detector_.setIntrinsics(rgb_intrinsics_matrix);

  // Get ground plane from parameters
  Eigen::Hyperplane<float, 3> groundPlane = getGroundPlane();
  // Transform groundplane into detection frame
  Eigen::Affine3d baseLinkToDetectionFrame = Eigen::Affine3d(
          Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX())*
          Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ()));

  Eigen::Hyperplane<float, 3> groundPlaneInDetectionFrame = groundPlane.transform(baseLinkToDetectionFrame.cast<float>());
  Eigen::VectorXf groundPlaneCoeffs(4);
  groundPlaneCoeffs(0) = 0.0;
  groundPlaneCoeffs(1) = 1.0;
  groundPlaneCoeffs(2) = 0.0;
  groundPlaneCoeffs(3) = -groundPlaneInDetectionFrame.offset(); // not sure why minus sign is needed
  people_detector_.setGround(groundPlaneCoeffs);

  ROS_INFO("Completed setup people detector object");
}

bool PCLPeopleDetection::compute()
{
  if (scene_cloud_->empty()){
    ROS_INFO("Inpuse empty cloud, skip this run");
    return false;
  }

  P_Clusters clusters;
  if(!people_detector_.compute(clusters)){
    ROS_INFO("Unable to compute the people cluster");
    return false;
  }

  // Apply minimum confidence level constraint
  P_Clusters clustersWithinConfidenceBounds;
  for (P_Clusters::iterator it = clusters.begin(); it != clusters.end(); ++it) {
    double confidence = it->getPersonConfidence();
    if (confidence > min_confidence_){             // draw only people with confidence above a threshold
      clustersWithinConfidenceBounds.push_back(*it);
    }
  }

  if(!clustersWithinConfidenceBounds.empty()){
    double angle = clustersWithinConfidenceBounds.front().getAngle();
    people_pose_.translation() = clustersWithinConfidenceBounds.front().getCenter();
    people_pose_.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
    return true;
  }
  else{
//    ROS_INFO("Does not detect people in the scene");
    return false;
  }
}

void PCLPeopleDetection::setInputCloud(const PointCloud &input_cloud)
{
  scene_cloud_.reset(new PointCloud(input_cloud));
  people_detector_.setInputCloud(scene_cloud_);
}

Eigen::Hyperplane<float, 3> PCLPeopleDetection::getGroundPlane()
{
  std::vector<float> original_ground_coeffs;
  original_ground_coeffs.push_back(0);
  original_ground_coeffs.push_back(0);
  original_ground_coeffs.push_back(1);
  original_ground_coeffs.push_back(0.8870);
//  original_ground_coeffs.push_back(0.8870); // 89cm

  return Eigen::Hyperplane<float, 3>(
          Eigen::Vector3f(original_ground_coeffs[0], original_ground_coeffs[1], original_ground_coeffs[2]),
          original_ground_coeffs[3]);
}

}
