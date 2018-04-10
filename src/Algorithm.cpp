#include "local_grid_map/Algorithm.hpp"

namespace local_grid_map {

Algorithm::Algorithm()
    :calib_img_size_(),
	  out_img_size_(),
	  gauss_ivar_()
{
}

Algorithm::~Algorithm()
{
}

cv::Mat Algorithm::generateDisparityMap(cv::Mat& left, cv::Mat& right){

  if (left.empty() || right.empty())
    return left;
  const cv::Size imsize = left.size();
  const int32_t dims[3] = {imsize.width,imsize.height,imsize.width};
  cv::Mat leftdpf = cv::Mat::zeros(imsize, CV_32F);
  cv::Mat rightdpf = cv::Mat::zeros(imsize, CV_32F);

  Elas::parameters param(Elas::ROBOTICS);

  param.postprocess_only_left = true;

  Elas elas(param);

  elas.process(left.data,right.data,leftdpf.ptr<float>(0), rightdpf.ptr<float>(0),dims);

  cv::Mat dmap = cv::Mat(out_img_size_, CV_8UC1, cv::Scalar(0));
  leftdpf.convertTo(dmap, CV_8U, 1.);

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

sensor_msgs::PointCloud Algorithm::processPointCloud(cv::Mat& img, cv::Mat& dmap, int ncells){

  cv::Mat V = cv::Mat(4, 1, CV_64FC1);
  cv::Mat pos = cv::Mat(4, 1, CV_64FC1);
  std::vector< cv::Point3d > points;

  sensor_msgs::ChannelFloat32 ch;
  sensor_msgs::PointCloud pc;

  ch.name = "rgb";
  pc.header.frame_id = "camera0";
  pc.header.stamp = ros::Time::now();


  for (int i = 0; i < img.cols; i++){
    for (int j = 0; j < img.rows; j++){
      int d = dmap.at<uchar>(j,i);

      // ignore low disparity point
      if (d < 10)
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

      // cv::Mat point3d_robot = XR_ * point3d_cam + XT_;

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

void Algorithm::getTransform(const std::string target_frame, const std::string source_frame){
  try{
    listener_.lookupTransform(target_frame, source_frame,
                             ros::Time(0), transform_);
  }
  catch (tf::TransformException & ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("Camera origin: x= %f, y = %f, z = %f", transform_.getOrigin().x()
           , transform_.getOrigin().y(), transform_.getOrigin().z());


  // Test transformation
  tf::Vector3 point;
  tf::Vector3 v(0,0,3);
  point = transform_.getBasis() * v + transform_.getOrigin();

  ROS_INFO("Transform test: x= %f, y = %f, z = %f", point.x(),point.y(), point.z());

}

float Algorithm::getElevation(grid_map::Position position, sensor_msgs::PointCloud pc){

  double w_num, w_fact, pos_x, pos_y, d, dx, dy, beta;


  tf::Vector3 pc_map;

  w_num   = 0.0;
  w_fact  = 0.0;

  for(int i = 0; i<pc.points.size(); i++){

    tf::Vector3 v_pc(pc.points[i].x,pc.points[i].y,pc.points[i].z);

    pc_map = transform_.getBasis() * v_pc + transform_.getOrigin();

    d = sqrt(pow(pc_map.x()-position.x(),2)+pow(pc_map.y()-position.y(),2));

    if(d < 0.5){
      // exp(x) = 1 + x(1+x/2(1+(x/3)(...))) ~ 1 + x (1 + x/2)
      beta = exp(-d*gauss_ivar_);
      // beta = 1 - d*(1 - d/2);
      w_num += beta*pc_map.z();
      w_fact += beta;
    }
  }

  if(w_fact == 0)
    return 0.0;
  return w_num / w_fact;
}

void Algorithm::setGaussParameter(double gauss_var){
  gauss_ivar_ = 1/(2*pow(gauss_var,2));
}

} /* namespace */
