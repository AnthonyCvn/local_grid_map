#include "local_grid_map/LocalGridMap.hpp"

#include <string>
#include <cstdint>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace local_grid_map {

LocalGridMap::LocalGridMap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // Launch the ImageTransport publisher
  image_transport::ImageTransport it(nodeHandle);

  imageSubscriber_ = it.subscribe(imageTopic_, 1,
		  	  	  	  	  	  	  	  &LocalGridMap::imageCallback, this ,
									  image_transport::TransportHints("theora",
											   ros::TransportHints().unreliable()));

  // Launch the ROS service.
  serviceServer_ = nodeHandle_.advertiseService("trigger",
                                                &LocalGridMap::serviceCallback, this);

  // Launch the ROS timer interruption.
  timer_ = nodeHandle_.createTimer(ros::Duration(1/(double)publishRate_),
		  	  	  	  	  	  	   &LocalGridMap::timerCallback, this);

  ROS_INFO("Node successfully launched.");
}

LocalGridMap::~LocalGridMap()
{
}

bool LocalGridMap::readParameters()
{
  if (nodeHandle_.getParam("map_frame_id", mapFrameId_) 	&&
	  nodeHandle_.getParam("image_topic", imageTopic_) 		&&
	  nodeHandle_.getParam("publish_rate", publishRate_)) return true;
  return false;
}

void LocalGridMap::timerCallback(const ros::TimerEvent& event)
{
	// ...

}

bool LocalGridMap::serviceCallback(std_srvs::Trigger::Request& request,
                                   std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(3);
  return true;
}

void LocalGridMap::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // ...

}

} /* namespace */
