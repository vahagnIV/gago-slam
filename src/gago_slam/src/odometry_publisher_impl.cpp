//
// Created by vahagn on 11/11/2021.
//

#include "odometry_publisher_impl.h"
#include <std_msgs/UInt8MultiArray.h>

namespace orb_slam3 {
namespace ros_publisher {

OdometryPublisherImpl::OdometryPublisherImpl(ros::NodeHandle & n, const std::string & topic) : thread_(nullptr) {
  publisher_ = n.advertise<std_msgs::UInt8MultiArray>(topic, 32);
}

OdometryPublisherImpl::~OdometryPublisherImpl() {
  Stop();
}

void OdometryPublisherImpl::Stop() {
  if (nullptr == thread_)
    return;
  cancelled_ = true;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void OdometryPublisherImpl::Start() {
  if (nullptr != thread_)
    return;

  cancelled_ = false;
  thread_ = new std::thread(&OdometryPublisherImpl::Worker, this);
}

void OdometryPublisherImpl::Worker() {
  while (!cancelled_) {
    messages::BaseMessage * message;
    if (messages::MessageProcessor::Instance().Dequeue(message)) {

      std_msgs::UInt8MultiArray ros_msg;

      message->Serialize(ros_msg.data);

      ros_msg.layout.data_offset = 0;
      ros_msg.layout.dim.resize(1);
      ros_msg.layout.dim[0].size = ros_msg.data.size();
      ros_msg.layout.dim[0].stride = 1;
      ros_msg.layout.dim[0].label = messages::to_string(message->Type());

      delete message;

      publisher_.publish(ros_msg);

    } else
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

  }
}

}
}