#pragma once

// Package algorithm
#include "local_grid_map/Algorithm.hpp"

// STD
#include <iostream>
#include <string>
#include <cstdint>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>

// Dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <local_grid_map/GridMapParamsConfig.h>

// Image transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>

// Grid map
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// OpenCv and ROS bridge
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// LIBELAS (Library for Efficient Large-scale Stereo Matching)
#include "elas.h"

// Eigen (Linear algebra)
#include "Eigen/Eigen"

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
  LocalGridMap(ros::NodeHandle& nodeHandle, std::string imageTopicL, std::string imageTopicR);

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
   * ROS dynamic reconfiguration callback function.
   * @param config parameter.
   * @param level.
   */
  void configcallback(const local_grid_map::GridMapParamsConfig &config, const uint32_t& level);
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
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Point cloud publisher.
  ros::Publisher pointCloud_pub_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Image transport subscriber
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter sub_img_left_;
  image_transport::SubscriberFilter sub_img_right_;

  //! Message filters synchronization.
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer< SyncPolicy > sync_;

  //! Dynamic reconfiguration server
  dynamic_reconfigure::Server<local_grid_map::GridMapParamsConfig> reconfig_server_;
  dynamic_reconfigure::Server<local_grid_map::GridMapParamsConfig>::CallbackType f_configcallback_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! ROS timer.
  ros::Timer timer_;

  //! Algorithm computation object.
  Algorithm algorithm_;

  //! UVC camera capture.
  cv::VideoCapture capture_;

  //! ROS server parameters; Frame identification.
  std::string mapFrameId_;
  std::string camFrameId_;

  //! Image topic to subscribe.
  std::string imageTopicL_;
  std::string imageTopicR_;

  //! ROS server parameters; Publish rate.
  int publishRate_;

  //! Remap output image size.
  int out_img_height_;
  int out_img_width_;

  //! Resolution of the grid map.
  double resolution_;

  //! Length of the map
  double mapLengthX_;
  double mapLengthY_;

  //! Range of the cell's value.
  double minHeight_;
  double maxHeight_;

  //! Initialized logic.
  bool mapInitialized_;

  //! Path of file storage with camera parameters.
  std::string calib_file_path_;

  //! Number of cache layer;
  int map_cache_number_;

  //! Queue size for image topics synchronization.
  int  sync_queue_size_;

};

} /* namespace */
