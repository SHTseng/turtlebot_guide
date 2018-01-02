#ifndef MESH_MODEL_LOADER_H
#define MESH_MODEL_LOADER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <string>

namespace turtlebot_guide
{

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

enum file_type{stl, obj, ply};

class MeshModelLoader
{
public:
  MeshModelLoader(const std::string& file_name);

  ~MeshModelLoader();

  bool readFile(PointCloud &cloud);

  void convertMeshToPCD();

private:
  std::string file_name_;
  file_type file_type_;
};
}

#endif // MESH_MODEL_LOADER_H
