#include "local_grid_map/Algorithm.hpp"

namespace local_grid_map {

Algorithm::Algorithm()
    : istoconf_(false),
	  isrunning_(true),
	  calib_img_size_(),
	  out_img_size_()
{
}

Algorithm::~Algorithm()
{
}

cv::Mat Algorithm::generateDisparityMap(cv::Mat& left, cv::Mat& right){

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

  cv::Mat dmap = cv::Mat(imsize, CV_8UC1, cv::Scalar(0));

  cv::normalize(leftdpf, dmap, 0, 255, cv::NORM_MINMAX, CV_8U);

  cv::resize(dmap, dmap, cv::Size(imsize.width/2, imsize.height/2), 0, 0, cv::INTER_AREA);// Or cv::INTER_CUBIC

  //leftdpf.convertTo(show, CV_8U, 1.);
  //cv::imshow("leftdpf",leftdpf);

  // copy float to uchar
  //for (int32_t i=0; i<imsize.width*imsize.height; i++) {
    //show.data[i] = (uint8_t)std::max(255.0*leftdpf.data[i]/disp_max,0.0);
  //}

  //imshow("FLOAT", leftdpf);

  return dmap;
}

bool Algorithm::loadCameraParameters(std::string filename){
  cv::FileStorage calib_file(filename, cv::FileStorage::READ);
  calib_file.open(filename, cv::FileStorage::READ);

  if(!(calib_file.isOpened()))
    return false;

  calib_file["K1"] >> K1_;
  calib_file["K2"] >> K2_;
  calib_file["D1"] >> D1_;
  calib_file["D2"] >> D2_;
  calib_file["R"]  >> R_;
  calib_file["T"]  >> T_;
  calib_file["XR"] >> XR_;
  calib_file["XT"] >> XT_;

  calib_file.release();

  return true;
}

void Algorithm::setIOImageSize(cv::Size image_size_calib, cv::Size image_size_out){
  calib_img_size_ = image_size_calib;
  out_img_size_ = image_size_out;
}

void Algorithm::findRectificationMap(){
  cv::Rect validRoi[2];

  ROS_INFO("Build rectification map");


  stereoRectify(K1_, D1_, K2_, D2_, calib_img_size_, R_, cv::Mat(T_), R1_, R2_, P1_, P2_, Q_,
                CV_CALIB_ZERO_DISPARITY, 0, out_img_size_, &validRoi[0], &validRoi[1]);

  cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_, out_img_size_, CV_32F, lmapx_, lmapy_);
  cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_, out_img_size_, CV_32F, rmapx_, rmapy_);

  ROS_INFO("Done rectification map ");

}

void Algorithm::remapImage(cv::Mat& src, cv::Mat& dst, setting side){
  if (side == LEFT) {
    cv::remap(src, dst, lmapx_, lmapy_, cv::INTER_LINEAR);
  } else {
    cv::remap(src, dst, rmapx_, rmapy_, cv::INTER_LINEAR);
  }
}

sensor_msgs::PointCloud Algorithm::processPointCloud(cv::Mat& img, cv::Mat& dmap){
  cv::Mat V = cv::Mat(4, 1, CV_64FC1);
  cv::Mat pos = cv::Mat(4, 1, CV_64FC1);
  std::vector< cv::Point3d > points;

  sensor_msgs::PointCloud pc;
  sensor_msgs::ChannelFloat32 ch;

  ch.name = "rgb";
  pc.header.frame_id = "world";
  pc.header.stamp = ros::Time::now();

  for (int i = 0; i < img.cols; i++){
    for (int j = 0; j < img.rows; j++){
      int d = dmap.at<uchar>(j,i);

      // ignore low disparity point
      if (d < 2)
        continue;

      // 3D homogenous coordinates of the image point
      V.at<double>(0,0) = (double)(i);
      V.at<double>(1,0) = (double)(j);
      V.at<double>(2,0) = (double)d;
      V.at<double>(3,0) = 1.;

      // 3D homogeneous coordinate = Q_ * V
      pos = Q_ * V;

      // Homogeneous to cartesian coordinate.
      double X = pos.at<double>(0,0) / pos.at<double>(3,0);
      double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
      double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
      cv::Mat point3d_cam = cv::Mat(3, 1, CV_64FC1);
      point3d_cam.at<double>(0,0) = X;
      point3d_cam.at<double>(1,0) = Y;
      point3d_cam.at<double>(2,0) = Z;

      //
      points.push_back(cv::Point3d(point3d_cam));
      geometry_msgs::Point32 pt;
      pt.x = point3d_cam.at<double>(0,0);
      pt.y = point3d_cam.at<double>(1,0);
      pt.z = point3d_cam.at<double>(2,0);
      pc.points.push_back(pt);
      int32_t red, blue, green;
      red = img.at<cv::Vec3b>(j,i)[2];
      green = img.at<cv::Vec3b>(j,i)[1];
      blue = img.at<cv::Vec3b>(j,i)[0];
      int32_t rgb = (red << 16 | green << 8 | blue);
      ch.values.push_back(*reinterpret_cast<float*>(&rgb));

    }
  }
  pc.channels.push_back(ch);
  return pc;
}



void Algorithm::setCamSettings(const CamSettings cam_settings){
  camsettings_ = cam_settings;
}

CamSettings Algorithm::getCamSettings(){
  return camsettings_;
}

void Algorithm::setCamIsToConf(const bool conf_camera){
  istoconf_ = conf_camera;
}

bool Algorithm::getCamIsToConf()
{
  if(istoconf_)
  {
    istoconf_ = false;
	return true;
  }
  return false;
}

bool Algorithm::getCamIsRunning()
{
  return isrunning_;
}

void Algorithm::setCamIsRunning(const bool run_camera)
{
	isrunning_ = run_camera;

}

} /* namespace */
