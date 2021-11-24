//
// Created by vahagn on 24/11/2021.
//

#include "camera_watcher.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

CameraWatcher::CameraWatcher(image_transport::ImageTransport & it,
                             const std::string & topic_name) : publisher_(it.advertise(topic_name, 4)) {

}

void CameraWatcher::SetCameras(const std::vector<const gago::io::video::CameraMeta *> & out_cameras) {

}

void CameraWatcher::Notify(const std::shared_ptr<std::vector<gago::io::video::Capture>> & data) {
  sensor_msgs::Image img;
  img.data = (*data)[0].data;
  img.height = (*data)[0].height;
  img.width = (*data)[0].width;
  img.encoding = sensor_msgs::image_encodings::RGB8;
  img.step = img.width * 3;
  img.header.stamp = ros::Time::now();
  publisher_.publish(img);
}
