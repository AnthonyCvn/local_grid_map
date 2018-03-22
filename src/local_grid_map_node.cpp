#include <ros/ros.h>
#include "local_grid_map/LocalGridMap.hpp"

#include <image_transport/image_transport.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_grid_map");
  ros::NodeHandle nodeHandle("~");

  local_grid_map::LocalGridMap LocalGridMap(nodeHandle);

  ros::spin();

  return 0;
}
