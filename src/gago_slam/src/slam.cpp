//
// Created by vahagn on 24/11/2021.
//

#include "slam.h"

#include <sensor_msgs/image_encodings.h>

#include <orb_slam3/features/orb_feature_extractor.h>
#include <orb_slam3/factories/keyframe_database_factory.h>
#include <orb_slam3/camera/monocular_camera.h>
#include <orb_slam3/frame/monocular/monocular_frame.h>
#include <orb_slam3/camera/distortions/barrel5.h>
#include <orb_slam3/settings.h>

#include <opencv2/opencv.hpp>

Slam::Slam() : tracker_(nullptr) {
  sensor_constants_.min_mp_disappearance_count = 2;
  sensor_constants_.max_allowed_discrepancy = 3;
  sensor_constants_.number_of_keyframe_to_search_lm = 20;
  sensor_constants_.projection_search_radius_multiplier = 1.;
  sensor_constants_.projection_search_radius_multiplier_after_relocalization = 5.;
  sensor_constants_.projection_search_radius_multiplier_after_lost = 15.;
  sensor_constants_.min_number_of_edges_sim3_opt = 20;
  sensor_constants_.sim3_optimization_threshold = 10.;
  sensor_constants_.sim3_optimization_huber_delta = std::sqrt(10.);

  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::MAP_CREATED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::TRACKING_INFO);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::KEYFRAME_CREATED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::KEYFRAME_DELETED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::MAP_POINT_CREATED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::MAP_POINT_DELETED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::OBSERVATION_ADDED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::OBSERVATION_DELETED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::KEYFRAME_POSITION_UPDATED);
  orb_slam3::Settings::Get().RequestMessage(orb_slam3::messages::MessageType::MAP_POINT_GEOMETRY_UPDATED);

}

void Slam::Initialize(uint32_t width, uint32_t height) {
  orb_slam3::precision_t SCALE_FACTOR = 1.2;
  const size_t LEVELS = 8;
  const size_t INIT_THRESHOLD = 20;
  const size_t MIN_THRESHOLD = 7;
  auto feature_extractor = new orb_slam3::features::ORBFeatureExtractor(width,
                                                                        height,
                                                                        SCALE_FACTOR,
                                                                        LEVELS,
                                                                        INIT_THRESHOLD,
                                                                        MIN_THRESHOLD);

  auto kf_database =
      orb_slam3::factories::KeyframeDatabaseFactory::CreateKeyFrameDatabase(orb_slam3::frame::KeyframeDatabaseType::DBoW2DB);
  auto atlas = new orb_slam3::map::Atlas(feature_extractor, kf_database);
  auto lm_detector = new orb_slam3::LoopMergeDetector(atlas);
  auto local_mapper = new orb_slam3::LocalMapper(atlas, lm_detector);
  lm_detector->Start();
  local_mapper->Start();
  tracker_ = new orb_slam3::Tracker(atlas, local_mapper);

  camera_ = new orb_slam3::camera::MonocularCamera(width, height);
  camera_->SetFx(8.1005682009901898e+02);
  camera_->SetFy(8.1326697136266705e+02);
  camera_->SetCx(3.1570785258350969e+02);
  camera_->SetCy(2.3651013402407037e+02);
  auto distortion = new orb_slam3::camera::Barrel5();
  distortion->SetK1(8.5110481971506762e-03);
  distortion->SetK2(-6.6965861593217962e-02);
  distortion->SetP1(-1.3249888115612043e-03);
  distortion->SetP2(1.0420014277021194e-04);
  distortion->SetK3(0);
  camera_->SetDistortionModel(distortion);
}

void Slam::ToEigenMat(const sensor_msgs::ImageConstPtr & image, orb_slam3::TImageGray8U & out_result) {
  //0.299 ∙ Red + 0.587 ∙ Green + 0.114 ∙ Blue
  out_result.resize(image->height, image->width);
  for (size_t row = 0; row < image->height; ++row) {
    for (size_t col = 0; col < image->width; ++col) {
      size_t pixel_offset = (row * image->width + col) * 3;
      double r = image->data[pixel_offset];
      double g = image->data[pixel_offset + 1];
      double b = image->data[pixel_offset + 2];
      out_result(row, col) = 0.299 * r + 0.587 * g + 0.114 * b;
    }
  }

}

void Slam::ProcessImage(const sensor_msgs::ImageConstPtr & image) {
  static const std::string debug_folder = "/data/slam_test/";
  std::string filename = debug_folder +  std::to_string(image->header.stamp.toNSec()) + ".jpg";
  cv::Mat img(image->height, image->width, CV_8UC3, (void *)image->data.data());
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  cv::imwrite(filename, img);
  if (nullptr == tracker_) {
    Initialize(image->width, image->height);
  }



  if (image->encoding == sensor_msgs::image_encodings::RGB8) {
    orb_slam3::TImageGray8U eigen_image;
    ToEigenMat(image, eigen_image);
    static const orb_slam3::features::handlers::HandlerType
        kHandlerType = orb_slam3::features::handlers::HandlerType::DBoW2;

    typedef orb_slam3::frame::monocular::MonocularFrame MF;

    auto frame = new MF(tracker_->GetAtlas(),
                        kHandlerType,
                        tracker_->GetState() == orb_slam3::Tracker::OK ? 1500 : 7500,
                        eigen_image,
                        orb_slam3::TimePoint(std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::nanoseconds(
                            image->header.stamp.toNSec()))),
                        filename,
                        camera_,
                        &sensor_constants_);
    tracker_->Track(frame);


  }

}
