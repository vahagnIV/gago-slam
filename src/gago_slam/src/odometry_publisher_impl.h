//
// Created by vahagn on 12/11/2021.
//

#ifndef ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_ODOMETRY_PUBLISHER_IMPL_H_
#define ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_ODOMETRY_PUBLISHER_IMPL_H_

// ==== stl =====
#include <thread>

// ==== ros =====
#include <ros/ros.h>

// ==== orb_slam3 =====
#include "messages/messages.h"
#include "odometry_publisher.h"

namespace orb_slam3 {
namespace ros_publisher {

class OdometryPublisherImpl: public OdometryPublisher {
 public:
  OdometryPublisherImpl(ros::NodeHandle & n, const std::string & topic);
  void Start() override;
  void Stop() override;
  virtual ~OdometryPublisherImpl() override;
 private:
  void Worker();
  template<typename T>
  T * Convert(messages::BaseMessage * base_msg) {
    T * result = dynamic_cast<T *>(base_msg);
    assert(result != nullptr);
    return result;
  }

 private:
  ros::Publisher publisher_;
  std::thread * thread_;
  bool cancelled_;

};

}
}
#endif //ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_ODOMETRY_PUBLISHER_IMPL_H_
