//
// Created by vahagn on 24/11/2021.
//

#include <ros/ros.h>
#include <gago/io/video/linux/v4l_driver.h>
#include "camera_watcher.h"


int main(int argc, char * argv[]) {
  ros::init(argc, argv, "ImagePublishingNode");


  gago::io::video::V4lDriver driver;
  driver.Initialize();
  auto cameras = driver.GetCameras();

  if (cameras.empty()) {
    ROS_ERROR("No cameras to capture");
  }

  if (cameras.size() > 1) {
    ROS_INFO("Found more than one cameras. Camera count: %i. Will use the first one that successfully opens");
  }

  auto formats = cameras[0]->GetFormats();
  bool motion_jpeg = false;
  auto resolutions = cameras[0]->GetResolutions();
  for (int i = 0; i < formats.size(); ++i) {
    if (formats[i] == "Motion-JPEG") {
      motion_jpeg = true;
      gago::io::video::CameraSettings settings(cameras[0]);
      settings.config.format_index = i;
      settings.config.resolution_index = 0; //18;
      settings.config.name = "WorkingCamera";
      settings.config.resolution_index = 0;
      settings.config.vertical_flip = false;
      driver.SetSettings(std::vector<gago::io::video::CameraSettings>{settings});
    }
  }

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  auto camera_watcher = new CameraWatcher(it, "CameraImage");
  driver.RegisterWatcher(camera_watcher);
  driver.Start();

  ros::spin();
  driver.UnRegister(camera_watcher);
  return 0;

}