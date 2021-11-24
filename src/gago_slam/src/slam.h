//
// Created by vahagn on 24/11/2021.
//

#ifndef GAGO_SLAM_SRC_GAGO_SLAM_SRC_SLAM_H_
#define GAGO_SLAM_SRC_GAGO_SLAM_SRC_SLAM_H_

#include <sensor_msgs/Image.h>
#include <orb_slam3/tracker.h>
#include <orb_slam3/camera/monocular_camera.h>
class Slam {

 public:
  Slam();
  virtual ~Slam() = default;
  void ProcessImage(const sensor_msgs::ImageConstPtr & image);

 private:
  void Initialize(uint32_t width, uint32_t height);
  static void ToEigenMat(const sensor_msgs::ImageConstPtr & image, orb_slam3::TImageGray8U & out_result);

 private:
  orb_slam3::Tracker * tracker_;
  orb_slam3::camera::MonocularCamera * camera_;
  orb_slam3::frame::SensorConstants sensor_constants_;
};

#endif //GAGO_SLAM_SRC_GAGO_SLAM_SRC_SLAM_H_
