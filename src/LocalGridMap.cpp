#include "local_grid_map/LocalGridMap.hpp"

namespace local_grid_map {

LocalGridMap::LocalGridMap(ros::NodeHandle& nodeHandle, std::string imageTopicL, std::string imageTopicR)
    : nodeHandle_(nodeHandle),
      it_(nodeHandle),
      sub_img_left_(it_, imageTopicL, 1, image_transport::TransportHints("theora", ros::TransportHints().unreliable())),
      sub_img_right_(it_, imageTopicR, 1),
      sync_(SyncPolicy(10), sub_img_right_, sub_img_left_),
	    map_(grid_map::GridMap({"original", "elevation"})),
	    mapInitialized_(false)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }


  // Launch the ImageTransport subscriber for a single topic.
  /*
  imageSubscriber_ = it_.subscribe(imageTopicL_, 1,
		  	  	  	  	  	  	  	   &LocalGridMap::imageCallback, this ,
		  	  	  	  	  	  	  	   image_transport::TransportHints("raw",
		  	  	  	  	  	  	  	   ros::TransportHints().unreliable()));
   */


  // Synchronize images topic.
  sync_.registerCallback( boost::bind(&LocalGridMap::imageCallback, this, _1, _2 ) );

  // Launch the point cloud publisher.
  pointCloud_pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("/camera/left/point_cloud",1);

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
{ // Read ROS parameters
  if (!(nodeHandle_.getParam("map_frame_id", mapFrameId_) 	      &&
        nodeHandle_.getParam("image_topic_right", imageTopicR_) 	&&
        nodeHandle_.getParam("image_topic_left", imageTopicL_)    &&
        nodeHandle_.getParam("publish_rate", publishRate_)	      &&
        nodeHandle_.getParam("calib_file_path", calib_file_path_) &&
        nodeHandle_.getParam("resolution", resolution_)			      &&
        nodeHandle_.getParam("min_height", minHeight_)			      &&
        nodeHandle_.getParam("max_height", maxHeight_))) return false;

  // Load camera parameters
  return algorithm_.loadCameraParameters(calib_file_path_);
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

void LocalGridMap::imageCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
  // Compute disparity map
  cv::Mat img_left, img_right, img_left_color;
  cv::Mat tmpL = cv_bridge::toCvShare(msg_left, "mono8")->image;
  cv::Mat tmpR = cv_bridge::toCvShare(msg_right, "mono8")->image;

  cv::Size image_size_calib = cv::Size(tmpL.size().width, tmpL.size().height);
  cv::Size image_size_out = cv::Size(tmpL.size().width, tmpL.size().height);



  if (tmpL.empty() || tmpR.empty())
    return;

  if (!mapInitialized_){
      algorithm_.setIOImageSize(image_size_calib, image_size_out);
      algorithm_.findRectificationMap();
  }

  algorithm_.remapImage(tmpL, img_left, Algorithm::LEFT);
  algorithm_.remapImage(tmpR, img_right, Algorithm::RIGHT);

  cv::Mat dmap = algorithm_.generateDisparityMap(img_left, img_right);


  cvtColor(img_left, img_left_color, CV_GRAY2BGR);
  sensor_msgs::PointCloud pc;
  pc = algorithm_.processPointCloud(img_left_color, dmap);

  pointCloud_pub_.publish(pc);

    imshow("DISP", dmap);
//  imshow("LEFT", tmpL);
//  imshow("RECT-LEFT", img_left);
//  imshow("RECT-LEFT-COLOR", img_left_color);
//  imshow("RIGHT", tmpR);
//  imshow("RECT-RIGHT", img_right);
    cv::waitKey(30);

  if (!mapInitialized_) {
    //grid_map::GridMapRosConverter::initializeFromImage(current_msg, resolution_, map_);

    //grid_map::GridMapCvConverter::initializeFromImage(dmap,resolution_ ,map_);
    const double lengthX = resolution_ * dmap.rows;
    const double lengthY = resolution_ * dmap.cols;
    grid_map::Length length(lengthX, lengthY);
    map_.setGeometry(length, resolution_);

    // Grid map settings.
    map_.setFrameId(mapFrameId_);
    //map_.setGeometry(grid_map::Length(720, 2560), resolution_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
         map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));

    mapInitialized_ = true;
  }

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(dmap, "elevation", map_, minHeight_, maxHeight_);
  grid_map::GridMapCvConverter::addColorLayerFromImage<unsigned char, 3>(dmap, "color", map_);

  // Publish grid map.
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  gridMapPublisher_.publish(message);

}


} /* namespace */
