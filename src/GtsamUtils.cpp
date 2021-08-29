/**
 * @file GtsamUtils.cpp
 * @brief Utility functions for interacting with GTSAM objects.
 * @author Tonio Teran, teran@mit.edu
 * @author Alan Papalia, apapalia@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include "tonioviz/GtsamUtils.h"

#include <thread>  //NOLINT [build/c++11]

namespace mrg {

/* ************************************************************************** */
VizPose GetVizPose(const gtsam::Pose3& pose3, const double length,
                   const double width) {
  Eigen::Matrix4d pose = pose3.matrix();
  return std::make_tuple(pose, length, width);
}

/* ************************************************************************** */
VizPose GetVizPose(const gtsam::Pose2& pose2, const double length,
                   const double width) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d pose2mat = pose2.matrix();
  pose.block(0, 0, 2, 2) = pose2mat.block(0, 0, 2, 2);
  pose.block(0, 3, 2, 1) = pose2mat.block(0, 2, 2, 1);
  return std::make_tuple(pose, length, width);
}

/* ************************************************************************** */
VizLandmark GetVizLandmark(const gtsam::Point2& landmark) {
  VizLandmark v_landmark = Eigen::Vector3d::Zero();
  v_landmark.block(0, 0, 2, 1) = landmark.matrix();
  return v_landmark;
}

/* ************************************************************************** */
VizLandmark GetVizLandmark(const gtsam::Point3& landmark) {
  VizLandmark v_landmark = landmark.matrix();
  return v_landmark;
}

/* ************************************************************************** */
std::vector<VizLandmark> GetVizLandmarks(
    const std::vector<gtsam::Point2>& landmarks) {
  std::vector<VizLandmark> v_landmarks;
  v_landmarks.reserve(landmarks.size());
  for (const gtsam::Point2& gtsam_landmark : landmarks) {
    v_landmarks.emplace_back(GetVizLandmark(gtsam_landmark));
  }
  return v_landmarks;
}

/* ************************************************************************** */
std::vector<VizLandmark> GetVizLandmarks(
    const std::vector<gtsam::Point3>& landmarks) {
  std::vector<VizLandmark> v_landmarks;
  v_landmarks.reserve(landmarks.size());
  for (const gtsam::Point3& gtsam_landmark : landmarks) {
    v_landmarks.emplace_back(GetVizLandmark(gtsam_landmark));
  }
  return v_landmarks;
}

/* ************************************************************************** */
std::vector<VizPose> GetVizPoses(const gtsam::Values& values,
                                 const double length, const double width,
                                 const unsigned char c, const bool is3d) {
  std::vector<VizPose> vposes;
  vposes.reserve(values.size());

  size_t counter = 0;
  while (values.exists(gtsam::Symbol(c, counter))) {
    if (is3d) {  // gtsam::Pose3
      gtsam::Pose3 pose3 = values.at<gtsam::Pose3>(gtsam::Symbol(c, counter));
      vposes.push_back(GetVizPose(pose3, length, width));
    } else {  // gtsam::Pose2
      gtsam::Pose2 pose2 = values.at<gtsam::Pose2>(gtsam::Symbol(c, counter));
      vposes.push_back(GetVizPose(pose2, length, width));
    }
    counter++;
  }

  return vposes;
}

/* ************************************************************************** */
void VisualizeGtsamEstimates(
    const gtsam::Values& vals,
    const std::vector<std::vector<gtsam::Symbol>>& pose_symbols,
    const std::map<int, gtsam::Symbol>& landmark_symbols, const bool is3d,
    const bool animation, const size_t ms_wait) {
  // Create a sample visualizer object.
  mrg::VisualizerParams params;
  params.f = 600;
  params.mode = mrg::VisualizerMode::GRAPHONLY;
  params.kftype = mrg::KeyframeDrawType::kTriad;
  mrg::Visualizer viz{params};

  // Dispatch data playback loop and enter rendering loop.
  std::thread data_thread(GtsamDataLoop, &viz, vals, pose_symbols,
                          landmark_symbols, is3d, animation, ms_wait);

  viz.RenderWorld();
  data_thread.join();  // Wait until both threads are finished.
}

/* ************************************************************************** */
static void GtsamDataLoop(
    mrg::Visualizer* viz, const gtsam::Values& curr_estimate,
    const std::vector<std::vector<gtsam::Symbol>>& pose_symbols,
    const std::map<int, gtsam::Symbol>& landmark_symbols, const bool is3d,
    const bool animation, const size_t ms_wait) {
  gtsam::Symbol sym;

  // Add landmarks to the visualizer.
  for (const auto& [index, sym] : landmark_symbols) {
    if (is3d) {
      gtsam::Point3 point = curr_estimate.at<gtsam::Point3>(sym);
      viz->AddVizLandmark(mrg::GetVizLandmark(point));
    } else {
      gtsam::Point2 point = curr_estimate.at<gtsam::Point2>(sym);
      viz->AddVizLandmark(mrg::GetVizLandmark(point));
    }
  }

  // Add poses to the visualizer.
  size_t robot_index = 0;
  for (const std::vector<gtsam::Symbol>& robot_trajectory : pose_symbols) {
    for (const gtsam::Symbol& pose_symbol : robot_trajectory) {
      if (viz->HasForcedQuit()) return;
      if (is3d) {
        gtsam::Pose3 pose = curr_estimate.at<gtsam::Pose3>(pose_symbol);
        mrg::VizPose v_pose = mrg::GetVizPose(pose);
        viz->AddVizPose(v_pose, robot_index);
      } else {
        gtsam::Pose2 pose = curr_estimate.at<gtsam::Pose2>(pose_symbol);
        mrg::VizPose v_pose = mrg::GetVizPose(pose);
        viz->AddVizPose(v_pose, robot_index);
      }
    }

    // If animation pause before looping onto next timesteps.
    if (animation) {
      std::this_thread::sleep_for(std::chrono::milliseconds(ms_wait));
    }

    ++robot_index;
  }
}

}  // namespace mrg
