#include "local_grid_map/LocalGridMap.hpp"

namespace local_grid_map {

LocalGridMap::LocalGridMap(ros::NodeHandle& nodeHandle, std::string imageTopicL, std::string imageTopicR)
    : nodeHandle_(nodeHandle),
      it_(nodeHandle),
      reconfig_server_(nodeHandle),
      sub_img_left_(it_, imageTopicL, 1, image_transport::TransportHints("compressed", ros::TransportHints().unreliable())),
      sub_img_right_(it_, imageTopicR, 1),
      sync_(SyncPolicy(100), sub_img_right_, sub_img_left_),
	    mapInitialized_(false)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // Create the grid map and all layers.
  std::vector<std::string> layers(map_cache_number_ + 3);
  layers[0] = "mean";
  layers[1] = "variance_square";
  layers[2] = "elevation";
  for(int i=0;i<map_cache_number_;i++){
    layers[i+3] = "layer_"+std::to_string(i);
  }
  map_ = grid_map::GridMap(grid_map::GridMap(layers));

  // Synchronize images topic.
  sync_.registerCallback( boost::bind(&LocalGridMap::imageCallback, this, _1, _2 ) );

  // Dynamic reconfiguration server
  f_configcallback_ = boost::bind(&LocalGridMap::configcallback, this, _1, _2);
  reconfig_server_.setCallback(f_configcallback_);

  // Launch the point cloud publisher.
  pointCloud_pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud>(camFrameId_ + "/left/point_cloud",1);

  // Launch the GridMap publisher.
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

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
  // Read ROS parameters
  if (!(nodeHandle_.getParam("map_frame_id", mapFrameId_) 	        &&
        nodeHandle_.getParam("cam_frame_id", camFrameId_)           &&
        nodeHandle_.getParam("image_topic_right", imageTopicR_) 	  &&
        nodeHandle_.getParam("image_topic_left", imageTopicL_)      &&
        nodeHandle_.getParam("publish_rate", publishRate_)	        &&
        nodeHandle_.getParam("out_img_height", out_img_height_)          &&
        nodeHandle_.getParam("out_img_width", out_img_width_)          &&
        nodeHandle_.getParam("calib_file_path", calib_file_path_)   &&
        nodeHandle_.getParam("map_cache_number", map_cache_number_) &&
        nodeHandle_.getParam("map_length_x", mapLengthX_)           &&
        nodeHandle_.getParam("map_length_y", mapLengthY_)           &&
        nodeHandle_.getParam("resolution", resolution_)			        &&
        nodeHandle_.getParam("min_height", minHeight_)			        &&
        nodeHandle_.getParam("max_height", maxHeight_))) return false;

  // Load camera parameters.
  return algorithm_.loadCameraParameters(calib_file_path_);
}

void LocalGridMap::configcallback(const local_grid_map::GridMapParamsConfig &config, const uint32_t& level) {
  ROS_INFO("Reconfigure Request: %f", config.SIGMA);
  algorithm_.setGaussParameter(config.SIGMA);
}

void LocalGridMap::timerCallback(const ros::TimerEvent& event)
{
	// ...
}

bool LocalGridMap::serviceCallback(std_srvs::Trigger::Request& request,
                                   std_srvs::Trigger::Response& response)
{
  response.success = true;
  // ...
  return true;
}

void LocalGridMap::imageCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
  static const int ncells = mapLengthX_*mapLengthY_/(resolution_*resolution_);
  cv::Mat img_left, img_right, img_left_color;
  cv::Mat tmpL = cv_bridge::toCvShare(msg_left, "mono8")->image;
  cv::Mat tmpR = cv_bridge::toCvShare(msg_right, "mono8")->image;
  cv::Size image_size_calib = cv::Size(tmpL.size().width, tmpL.size().height);
  cv::Size image_size_out = cv::Size(out_img_width_, out_img_height_);

  if (tmpL.empty() || tmpR.empty())
    return;

  // Initialization based on pictures.
  if (!mapInitialized_){
    // Set algorithm parameters.
    algorithm_.setIOImageSize(image_size_calib, image_size_out);

    // Find rectification-distortion map.
    algorithm_.findRectificationMap();

    // Find TF transformation between camera and map frame.
    algorithm_.getTransform(mapFrameId_, camFrameId_);

    // Grid map settings.
    grid_map::Length length(mapLengthX_, mapLengthY_);
    map_.setGeometry(length, resolution_);
    map_.setFrameId(mapFrameId_);

    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
              map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));

      mapInitialized_ = true;
  }

  // Rectify and undistorted left and right image.
  algorithm_.remapImage(tmpL, img_left, Algorithm::LEFT);
  algorithm_.remapImage(tmpR, img_right, Algorithm::RIGHT);

  // Find rectification map with LIBELAS library.
  cv::Mat dmap = algorithm_.generateDisparityMap(img_left, img_right);

  // Generate point cloud.
  sensor_msgs::PointCloud pc;
  cvtColor(img_left, img_left_color, CV_GRAY2BGR);
  pc = algorithm_.processPointCloud(img_left_color, dmap, ncells, camFrameId_);

  // Add data to grid map layers.
  ros::Time time = ros::Time::now();
  static int n = 0;
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map_.getPosition(*it, position);
    map_.at("layer_"+std::to_string(n), *it) = algorithm_.getElevation(position, pc);
  }
  n++;
  if(n>=map_cache_number_)
    n=0;

  // Process the mean over layers.
  map_["mean"].setConstant(0.0);
  map_["variance_square"].setConstant(0.0);
  map_["elevation"].setConstant(0.0);

  for(int i=0;i<map_cache_number_;i++){
    map_["mean"] += map_["layer_"+std::to_string(i)];

  }
  map_["mean"] = 1.0/map_cache_number_ * map_["mean"];

  for(int i=0;i<map_cache_number_;i++){
    map_["variance_square"] += (map_["layer_"+std::to_string(i)] - map_["mean"]).cwiseProduct(map_["layer_"+std::to_string(i)] - map_["mean"]);

  }
  map_["variance_square"] = 1.0/(map_cache_number_-1) * map_["variance_square"];

  map_["elevation"] = map_["mean"];

  // Publish point cloud.
  pointCloud_pub_.publish(pc);

  // Publish grid map.
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  gridMapPublisher_.publish(message);
}


} /* namespace */
