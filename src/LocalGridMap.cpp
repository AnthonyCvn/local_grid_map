#include "local_grid_map/LocalGridMap.hpp"



namespace local_grid_map {

LocalGridMap::LocalGridMap(ros::NodeHandle& nodeHandle, std::string imageTopicL, std::string imageTopicR)
    : nodeHandle_(nodeHandle),
      it_(nodeHandle),
      sub_img_left_(it_, imageTopicL, 1, image_transport::TransportHints("theora", ros::TransportHints().unreliable())),
      sub_img_right_(it_, imageTopicR, 1, image_transport::TransportHints("theora", ros::TransportHints().unreliable())),
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

  // Launch the GridMap publisher
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
  if (nodeHandle_.getParam("map_frame_id", mapFrameId_) 	      &&
      nodeHandle_.getParam("image_topic_right", imageTopicR_) 	&&
      nodeHandle_.getParam("image_topic_left", imageTopicL_)    &&
      nodeHandle_.getParam("publish_rate", publishRate_)	      &&
      nodeHandle_.getParam("resolution", resolution_)			      &&
      nodeHandle_.getParam("min_height", minHeight_)			      &&
      nodeHandle_.getParam("max_height", maxHeight_)) return true;
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

void LocalGridMap::imageCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
  //ROS_INFO("0");
  // Compute disparity map
  cv::Mat tmpL, tmpR, tmp;
  cv::Mat lmapx, lmapy, rmapx, rmapy;

  tmpL = cv_bridge::toCvShare(msg_left, "mono8")->image;
  tmpR = cv_bridge::toCvShare(msg_right, "mono8")->image;

  //tmpL = cv_bridge::toCvShare(msg_left, "mono8")->image;
  //tmpR = cv_bridge::toCvShare(msg_right, "mono8")->image;

  if (tmpL.empty())
    return;

  //tmpL = tmp(cv::Rect(0, 0, tmp.cols / 2, tmp.rows));
  //tmpR = tmp(cv::Rect(tmp.cols /2, 0, tmp.cols / 2, tmp.rows));


  //cv::Mat img_left, img_right, img_left_color;

  // TODO Remaping with camera parameters (from calibration file)
  //cv::remap(tmpL, img_left, lmapx, lmapy, cv::INTER_LINEAR);
  //cv::remap(tmpR, img_right, rmapx, rmapy, cv::INTER_LINEAR);

  //img_left = tmpL;
  //img_right =tmpR;

  //cvtColor(img_left, img_left_color, CV_GRAY2BGR);

  //static int i = 0;
  //i++;
  //if(i>10){
    //i=0;
    const cv::Mat dmap = generateDisparityMap(tmpL, tmpR);


    imshow("DISP", dmap);
    imshow("LEFT", tmpL);
    imshow("RIGHT", tmpR);
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

cv::Mat LocalGridMap::generateDisparityMap(cv::Mat& left, cv::Mat& right){

  //int out_width = left.cols;
  //int out_height = left.rows;
  //cv::Size out_img_size = cv::Size(out_width, out_height);;

  //imshow("LEFT", left);
  //imshow("RIGHT", right);

  if (left.empty() || right.empty())
    return left;
  const cv::Size imsize = left.size();
  const int32_t dims[3] = {imsize.width,imsize.height,imsize.width};
  cv::Mat leftdpf = cv::Mat::zeros(imsize, CV_32F);
  cv::Mat rightdpf = cv::Mat::zeros(imsize, CV_32F);

  Elas::parameters param(Elas::MIDDLEBURY);

/*(Elas::ROBOTICS)
  param.disp_min              = 0;
  param.disp_max              = 255;
  param.support_threshold     = 0.85;
  param.support_texture       = 10;
  param.candidate_stepsize    = 5;
  param.incon_window_size     = 5;
  param.incon_threshold       = 5;
  param.incon_min_support     = 5;
  param.add_corners           = 0;
  param.grid_size             = 20;
  param.beta                  = 0.02;
  param.gamma                 = 3;
  param.sigma                 = 1;
  param.sradius               = 2;
  param.match_texture         = 1;
  param.lr_threshold          = 2;
  param.speckle_sim_threshold = 1;
  param.speckle_size          = 200;
  param.ipol_gap_width        = 3;
  param.filter_median         = 0;
  param.filter_adaptive_mean  = 1;
  param.postprocess_only_left = 1;
  param.subsampling           = 0;

*/

  param.postprocess_only_left = true;

  Elas elas(param);

  elas.process(left.data,right.data,leftdpf.ptr<float>(0), rightdpf.ptr<float>(0),dims);

  // find maximum disparity for scaling output disparity images to [0..255]
  float disp_max = 0;
  for (int32_t i=0; i<imsize.width*imsize.height; i++) {
    if (leftdpf.data[i]>disp_max) disp_max = leftdpf.data[i];
    if (rightdpf.data[i]>disp_max) disp_max = rightdpf.data[i];
  }

  cv::Mat show = cv::Mat(imsize, CV_8UC1, cv::Scalar(0));

  cv::normalize(leftdpf, show, 0, 255, cv::NORM_MINMAX, CV_8U);

  //leftdpf.convertTo(show, CV_8U, 1.);
  //cv::imshow("leftdpf",leftdpf);

  // copy float to uchar
  //for (int32_t i=0; i<imsize.width*imsize.height; i++) {
    //show.data[i] = (uint8_t)std::max(255.0*leftdpf.data[i]/disp_max,0.0);
  //}

  //imshow("FLOAT", leftdpf);

  return show;
}

} /* namespace */
