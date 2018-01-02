#include <turtlebot_follower_tracker/mesh_model_loader.h>

namespace turtlebot_guide
{

MeshModelLoader::MeshModelLoader(const std::string& file_name):
file_name_(file_name)
{
  file_name_ = ros::package::getPath("turtlebot_guide_integration")+"/robot/"+file_name_;
}

MeshModelLoader::~MeshModelLoader(){}

bool MeshModelLoader::readFile(PointCloud& cloud)
{
  if (pcl::io::loadPCDFile(file_name_, cloud) == -1){
    return false;
  }
  return true;
}

} // namespace
