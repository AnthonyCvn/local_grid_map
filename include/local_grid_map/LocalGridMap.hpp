#pragma once

#include "local_grid_map/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

namespace local_grid_map {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class LocalGridMap
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  LocalGridMap(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~LocalGridMap();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS timer callback.
   * @param TimerEvent the event structure of the timer.
   */
  void timerCallback(const ros::TimerEvent &event);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  /*!
   * ROS service server callback.
   * @param msg theora left image topic.
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Left image transport subscriber
  image_transport::Subscriber imageSubscriber_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! ROS timer.
  ros::Timer timer_;

  //! Algorithm computation object.
  Algorithm algorithm_;

  //! Camera settings wrapper.
  CamSettings camSettings_;

  //! UVC camera capture.
  cv::VideoCapture capture_;

  //! ROS server parameters; Frame identification.
  std::string mapFrameId_;

  //! ROS server parameters; left image topic.
  std::string imageTopic_;

  //! ROS server parameters; Publish rate.
  int publishRate_;
};

} /* namespace */
