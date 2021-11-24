//
// Created by vahagn on 24/11/2021.
//

#ifndef GAGO_SLAM_SRC_IMAGE_PUBLISHER_SRC_CAMERA_WATCHER_H_
#define GAGO_SLAM_SRC_IMAGE_PUBLISHER_SRC_CAMERA_WATCHER_H_

#include <gago/io/video/icamera_watcher.h>
#include <image_transport/image_transport.h>

class CameraWatcher : public gago::io::video::CameraWatcher {
 public:
  CameraWatcher(image_transport::ImageTransport & it,
                const std::string & topic_name);
  void SetCameras(const std::vector<const gago::io::video::CameraMeta *> & out_cameras) override;
  void Notify(const std::shared_ptr<std::vector<gago::io::video::Capture>> & data) override;

 private:
  image_transport::Publisher publisher_;

};

#endif //GAGO_SLAM_SRC_IMAGE_PUBLISHER_SRC_CAMERA_WATCHER_H_
