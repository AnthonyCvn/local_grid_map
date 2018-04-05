#include <ros/ros.h>
#include "local_grid_map/LocalGridMap.hpp"

#include <image_transport/image_transport.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_grid_map");
  ros::NodeHandle nodeHandle("~");

  std::string imageTopicL, imageTopicR;

  nodeHandle.getParam("image_topic_left", imageTopicL);
  nodeHandle.getParam("image_topic_right",imageTopicR);

  local_grid_map::LocalGridMap LocalGridMap(nodeHandle, imageTopicL, imageTopicR);

  ros::spin();

  return 0;
}
