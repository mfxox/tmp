#include <iostream>
#include <sstream>
#include <vector>
#include <signal.h>
#include <boost/timer.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

#include <flycapture/FlyCapture2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

FlyCapture2::Error error;
FlyCapture2::Camera camera;
FlyCapture2::CameraInfo camInfo;

// disconnect from ladybug with ctrl-c
void sigint(int sig) {

  ROS_INFO_STREAM("Disconnecting ..." );

  error = camera.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK) {
    ROS_ERROR_STREAM("FlyCapture2: Failed to disconnect from camera." );
  }

  camera.Disconnect();

  ros::shutdown();
}

// generate numbering name with base name and number
std::string numbering_name(std::string base_name, int num) {
  std::stringstream ss;
  ss << base_name << num;
  return ss.str();
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "ladybug_driver");

  // for disconnect from ladybug
  signal(SIGINT, sigint);

  // setup ros params
  double resize = 1.0;
  double frame_rate = 8.0;
  double delay_time = 0.3; 
  bool disable_top = false;
  bool front_only = false;
  std::string camera_frame_id_base = std::string("ladybug/camera_");
  std::string camera_name_base = std::string("ladybug/camera_");
  std::string camera_info_uri = std::string("");

  ros::NodeHandle p_nh("~");
  p_nh.param("resize", resize, resize);
  p_nh.param("frame_rate", frame_rate, frame_rate);
  p_nh.param("delay_time", delay_time, delay_time);
  p_nh.param("disable_top", disable_top, disable_top);
  p_nh.param("front_only", front_only, front_only);
  p_nh.param("camera_frame_id_base", camera_frame_id_base, camera_frame_id_base);
  p_nh.param("camera_name_base", camera_name_base, camera_name_base);
  p_nh.param("camera_info_uri", camera_info_uri, camera_info_uri);

  // setup ros publisher
  ros::NodeHandle nh;
  std::vector<image_transport::ImageTransport> its;
  std::vector<image_transport::CameraPublisher> cpubs;

  int imgs_size = (disable_top) ? 5 : 6;

  if (front_only) imgs_size = 1; 

  for (int i = 0; i < imgs_size; i++) {
    its.push_back(image_transport::ImageTransport(nh));
    cpubs.push_back(its[i].advertiseCamera(numbering_name(camera_frame_id_base, i), 1));
  }

  // setup ros camera info
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cims[imgs_size];
  for (int i = 0; i < imgs_size; i++) cims[i].reset(new camera_info_manager::CameraInfoManager(nh, numbering_name(camera_name_base, i), camera_info_uri));

  // connect the camera
  error = camera.Connect(0);
  if (error != FlyCapture2::PGRERROR_OK) {
    ROS_ERROR_STREAM("FlyCapture2: Failed to connect to camera." );
    return false;
  }

  // get the camera info and print it out
  error = camera.GetCameraInfo(&camInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    ROS_ERROR_STREAM("FlyCapture2: Failed to get camera info from camera." );
    return false;
  }

  ROS_INFO_STREAM(camInfo.vendorName << " " << camInfo.modelName << " "  << camInfo.serialNumber );

  // set format7 mode
  FlyCapture2::Format7ImageSettings settings;
  settings.mode = FlyCapture2::MODE_0;
  settings.offsetX = 0;
  settings.offsetY = 0;
  settings.width = 2528;
  settings.height = 12484;
  settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;

  error = camera.SetFormat7Configuration(&settings, static_cast<unsigned int>(32000));
  if (error != FlyCapture2::PGRERROR_OK) {
    ROS_ERROR_STREAM("FlyCapture2: Failed to set format7 settings." );
    return false;
  }

  // set frame rate
  FlyCapture2::Property prop_f;
  prop_f.type = FlyCapture2::FRAME_RATE;
  prop_f.onOff = true;
  prop_f.autoManualMode = false;
  prop_f.absControl = true;
  prop_f.absValue = frame_rate;

  error = camera.SetProperty(&prop_f);
  if (error != FlyCapture2::PGRERROR_OK) {
    ROS_ERROR_STREAM("FlyCapture2: Failed to set frame rate." );
    return false;
  }

  // start capture
  error = camera.StartCapture();
  if (error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
    ROS_ERROR_STREAM("FlyCapture2: Bandwidth exceeded" );
    return false;
  } else if (error != FlyCapture2::PGRERROR_OK) {
    ROS_ERROR_STREAM("FlyCapture2: Failed to start image capture." );
    return false;
  }

  // capture loop
  while (ros::ok()) {
  
    // get the image
    FlyCapture2::Image rawImage;
    FlyCapture2::Error error = camera.RetrieveBuffer(&rawImage);
    if (error != FlyCapture2::PGRERROR_OK) {
      ROS_ERROR_STREAM("FlyCapture2: Capture error." );
      continue;
    }

    // get ros time
    ros::Time ts_now = ros::Time::now();
    ros::Time ts = ts_now - ros::Duration(delay_time);
    // convert to rgb
    FlyCapture2::Image rgbImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);
    
    // convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
    cv::Mat img_raw = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);

    // resize for calc
    cv::Mat img_rs;
    if (resize != 1.0) {
      img_rs = cv::Mat(img_raw.rows*resize, img_raw.cols*resize, img_raw.type());
      cv::resize(img_raw, img_rs, cv::Size(), resize, resize);    
    } else {
      img_rs = img_raw;
    }
    
    // transpose & flip
    cv::Mat img_t(img_rs.cols*resize, img_rs.rows*resize, img_rs.type());
    cv::flip(img_rs.t(), img_t, 1);
    
    // extract & publish
    cv::Mat img[imgs_size];
    for (int i = 0; i < imgs_size; i++) {

      img[i] = img_t(cv::Rect((10420 - 2058*i - 22*i)*resize, 36*resize, 2058*resize, 2456*resize));

      cv_bridge::CvImage msg;
      msg.header.stamp = ts;
      msg.header.frame_id = numbering_name(camera_frame_id_base, i);
      msg.encoding = sensor_msgs::image_encodings::BGR8;
      msg.image = img[i];

      sensor_msgs::CameraInfo ci(cims[i]->getCameraInfo());
      ci.header.stamp = msg.header.stamp;
      ci.header.frame_id = msg.header.frame_id;

      cpubs[i].publish(*msg.toImageMsg(), ci);
    }
    
    ros::spinOnce();
  }

  return 0;
}
