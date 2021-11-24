//
// Created by vahagn on 24/11/2021.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include "slam.h"
#include <orb_slam3/logging.h>

int main(int argc, char * argv[]) {
  ros::init(argc, argv, "GagoSlamNode");
  orb_slam3::logging::Initialize();
  const std::string raw_image_topic = "CameraImage";

  ros::NodeHandle node_handle;
  image_transport::ImageTransport it(node_handle);
  Slam slam;
  image_transport::Subscriber sub = it.subscribe(raw_image_topic ,8, &Slam::ProcessImage, &slam);
  ros::spin();

}