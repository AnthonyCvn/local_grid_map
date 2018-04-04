  #pragma once

// STD
#include <iostream>
#include <string>
#include <cstdint>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>

// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>

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
 * Structure containing the parameters of the camera.
 */
typedef struct
{
	int height;
	int width;
	int fps;
} CamSettings;

///*!
// * Structure containing the lookup table.
// */
//typedef struct
//{
//  int* data;
//} LookUpTable;

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm
{
 public:
  /*!
   * Enumeration settings
   */
  enum setting {LEFT, RIGHT};

  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  /*!
   * Generate disparity map with LIBELAS
   * @param left stereo image.
   * @param right stereo image.
   * @return disparity map.
   */
  cv::Mat generateDisparityMap(cv::Mat& left, cv::Mat& right);

  /*!
   * Load the camera extrinsic and intrinsic parameters.
   * @param filename path of the YAML calibration file.
   * @return if the file was open correctly.
   */
  bool loadCameraParameters(std::string filename);

  /*!
   * Find the rectification map.
   * @param finalSize size of the rectification map.
   */
  void findRectificationMap();

  /*!
   * Set size of the image used for stereo calibration and
   * of the output undistorted image after rectification.
   * @param camSettings wraps the settings of the camera.
   */
  void setIOImageSize(cv::Size image_size_calib, cv::Size image_size_out);

  /*!
   * Undistorts and rectifies the image with the homographic mappings
   * found with the findRectificationMap function.
   * @param src source image.
   * @param dst destination image.
   * @param left image if true and right if false.
   */
  void remapImage(cv::Mat& src, cv::Mat& dst, setting side);

  /*!
   * Calculate point cloud.
   * @param img image matrix.
   * @param dmap disparity map.
   * @return point cloud.
   */
  sensor_msgs::PointCloud processPointCloud(cv::Mat& img, cv::Mat& dmap, int ncells);

  /*!
   * Calculate point cloud.
   * @param target_frame
   * @param source_frame
   */
  void getTransform(const std::string target_frame, const std::string source_frame);

  /*!
   * Get the elevation of the map according to the position of the center of a cell.
   * @param position
   */
  float getElevation(grid_map::Position position, sensor_msgs::PointCloud pc);

  /*!
   * Set the camera settings parameters.
   * @param camSettings wraps the settings of the camera.
   */
  void setCamSettings(const CamSettings cam_settings);

  /*!
   * Get the camera settings parameters.
   * @return Settings of the camera.
   */
  CamSettings getCamSettings();

  /*!
   * Set the camera configuration request.
   * @param conf_camera set true for a new configuration.
   */
  void setCamIsToConf(const bool is_toconf);

  /*!
   * Check if the camera should be configured.
   * @return if the camera should be configured.
   */
  bool getCamIsToConf();

  /*!
   * Start or stop the camera.
   * @param run_camera set true to run the camera.
   */
  void setCamIsRunning(const bool is_running);

  /*!
   * Check if the camera should run.
   * @return if the camera should run.
   */
  bool getCamIsRunning();

  /*!
   * Set stereo mode.
   * @param run_camera set true to run the camera.
   */
  void setCamIsStereo(const bool is_stereo);

  /*!
   * Check if the camera is in stereo mode.
   * @return if the camera is in stereo mode.
   */
  bool getCamIsStereo();

 private:

//  //! Look up table.
//  int** lookup_;


  //! Internal logical variable to configure the camera only on service request.
  bool istoconf_;

  //! Internal logical variable to run the camera.
  bool isrunning_;

  //! Internal camera settings wrapper
  CamSettings camsettings_;

  //! Camera parameters._
  cv::Mat Q_, P1_, P2_;
  cv::Mat R1_, R2_, K1_, K2_, D1_, D2_, R_;
  cv::Mat lmapx_, lmapy_, rmapx_, rmapy_;
  cv::Vec3d T_;

  // Size of the image used for stereo calibration.
  cv::Size calib_img_size_;

  // Size undistorted and rectify image.
  cv::Size out_img_size_;

  // TF listener and transform
  tf::TransformListener listener_;
  tf::StampedTransform transform_;

//  //! Initialized logic.
//  bool initialized_;


};

} /* namespace */
