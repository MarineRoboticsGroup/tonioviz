/**
 * @file GtsamUtils.cpp
 * @brief Utility functions for interacting with GTSAM objects.
 * @author Tonio Teran, teran@mit.edu
 * @author Alan Papalia, apapalia@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#ifndef TONIOVIZ_GTSAMUTILS_H_
#define TONIOVIZ_GTSAMUTILS_H_

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>

#include <string>
#include <vector>

#include "tonioviz/DataUtils.h"
#include "tonioviz/Visualizer.h"

namespace mrg {

/**
 * @brief Takes GTSAM pose object and returns as VizPose object
 *
 * @param pose3     GTSAM pose to convert to matrix form
 * @param length    length of the pose axes
 * @param width     width of the pose axes
 * @return VizPose
 */
inline VizPose GetVizPose(const gtsam::Pose3 pose3, const double length = 0.1,
                          const double width = 2.0);

/**
 * @brief Takes GTSAM pose object and returns as VizPose object
 *
 * @param pose2     GTSAM pose to convert to matrix form
 * @param length    length of the pose axes
 * @param width     width of the pose axes
 * @return VizPose
 */
inline VizPose GetVizPose(const gtsam::Pose2 pose2, const double length = 0.1,
                          const double width = 2.0);

inline VizLandmark GetVizLandmark(const gtsam::Point2 landmark) {
  VizLandmark v_landmark = Eigen::Vector3d::Zero();
  v_landmark.block(0, 0, 2, 1) = landmark.matrix();
  return v_landmark;
}

/**
 * @brief Convert from gtsam::Point3 to VizLandmark
 *
 * @param landmark
 * @return VizLandmark
 */
inline VizLandmark GetVizLandmark(const gtsam::Point3 landmark) {
  VizLandmark v_landmark = landmark.matrix();
  return v_landmark;
}

/**
 * @brief Convert from vector of gtsam::Point2 to vector of VizLandmarks
 *
 * @param landmarks
 * @return std::vector<VizLandmark>
 */
inline std::vector<VizLandmark> GetVizLandmarks(
    const std::vector<gtsam::Point2> landmarks) {
  std::vector<VizLandmark> v_landmarks;
  for (size_t i = 0; i < landmarks.size(); i++) {
    v_landmarks.emplace_back(GetVizLandmark(landmarks[i]));
  }
  return v_landmarks;
}


/**
 * @brief Convert from vector of gtsam::Point3 to vector of VizLandmarks
 *
 * @param landmarks
 * @return std::vector<VizLandmark>
 */
inline std::vector<VizLandmark> GetVizLandmarks(
    const std::vector<gtsam::Point3> landmarks) {
  std::vector<VizLandmark> v_landmarks;
  for (size_t i = 0; i < landmarks.size(); i++) {
    v_landmarks.emplace_back(GetVizLandmark(landmarks[i]));
  }
  return v_landmarks;
}

/**
 * @brief Builds vector of visualization poses to pass directly onto visualizer.
 * @param[in] values   GTSAM struct with the poses to extract.
 * @param[in] length   Length of the pose axes.
 * @param[in] width    Width of the pose axes.
 * @param[in] c        Character with which poses were stored inside `values`.
 * @param[in] is3d     Whether to deal with Pose3 (otherwise assumes Pose2).
 *
 * Assumptions: the poses were store with increasing indices, starting from 0,
 * and without skipping an number.
 */
std::vector<VizPose> GetVizPoses(const gtsam::Values& values,
                                 const double length = 0.1,
                                 const double width = 2.0,
                                 const unsigned char c = 'x',
                                 const bool is3d = true);

/**
 * @brief Runs the visualizer in a separate thread. Constructs the visualizer
 * and then offloads the data loading to GtsamDataLoop
 *
 * @param vals                  the estimates of the poses and landmarks
 * @param pose_symbols          GTSAM symbols for all poses
 * @param landmark_symbols      GTSAM symbols for all landmarks
 * @param is3d                  whether the data is in 2D or 3D
 * @param animation             whether to play as animation
 * @param ms_wait               how long to pause (in ms) between frames
 *
 * NOTE: this assumes that all information provided is either a sequence of
 * poses or a vector of landmarks.
 *
 */
void VisualizeGtsamEstimates(
    const gtsam::Values vals,
    std::vector<std::vector<gtsam::Symbol>> pose_symbols,
    std::map<int, gtsam::Symbol> landmark_symbols, const bool is3d = true,
    const bool animation = true, const size_t ms_wait = 50);

/**
 * @brief Reads data in gtsam estimate and plays poses inside of visualizer. Can
 * be run as an animation (incrementally displaying poses) or just to show all
 * trajectories at once
 *
 * @param viz                   Visualization window
 * @param curr_estimate         the estimates of the poses and landmarks
 * @param pose_symbols          GTSAM symbols for all poses
 * @param landmark_symbols      GTSAM symbols for all landmarks
 * @param is3d                  whether the data is in 2D or 3D
 * @param animation             whether to play as animation
 * @param ms_wait               how long to pause between frames
 */
static void GtsamDataLoop(mrg::Visualizer* viz, gtsam::Values curr_estimate,
                          std::vector<std::vector<gtsam::Symbol>> pose_symbols,
                          std::map<int, gtsam::Symbol> landmark_symbols,
                          const bool is3d, const bool animation,
                          const size_t ms_wait);

}  // namespace mrg

#endif  // TONIOVIZ_GTSAMUTILS_H_
