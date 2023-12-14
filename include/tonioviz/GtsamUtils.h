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

#include <map>
#include <string>
#include <vector>

#include "tonioviz/DataUtils.h"
#include "tonioviz/Visualizer.h"

namespace mrg {

/**
 * @brief Takes GTSAM pose object and returns as VizPose object.
 *
 * @param pose3     GTSAM pose to convert to matrix form.
 * @param length    Length of the pose axes.
 * @param width     Width of the pose axes.
 * @return VizPose
 */
VizPose GetVizPose(const gtsam::Pose3& pose3, const double length = 0.1,
                   const double width = 2.0);

/**
 * @brief Takes GTSAM pose object and returns as VizPose object.
 *
 * @param pose2     GTSAM pose to convert to matrix form
 * @param length    Length of the pose axes.
 * @param width     Width of the pose axes.
 * @return VizPose
 */
VizPose GetVizPose(const gtsam::Pose2& pose2, const double length = 0.1,
                   const double width = 2.0);

/**
 * @brief Takes GTSAM 2D point and returns a corresponding 3D landmark vector.
 *
 * @param landmark  GTSAM 2D point.
 * @return Corresponding 3D point on the XY plane (Z = 0).
 */
VizLandmark GetVizLandmark(const gtsam::Point2& landmark);

/**
 * @brief Convert from gtsam::Point3 to VizLandmark.
 *
 * @param landmark  GTSAM 3D point.
 * @return Corresponding 3D point.
 */
VizLandmark GetVizLandmark(const gtsam::Point3& landmark);

/**
 * @brief Convert from gtsam::Point3 and Color to ColoLandmark.
 *
 * @param landmark  GTSAM 3D point.
 * @param color     Color of the landmark.
 * @return Corresponding 3D point.
 */
ColorLandmark GetColorLandmark(const gtsam::Point3& landmark, const Color& color);

/**
 * @brief Convert from vector of gtsam::Point2 to vector of VizLandmarks.
 *
 * @param landmarks  Vector of GTSAM 2D points.
 * @return Vector of corresponding 3D points on the XY plane (Z = 0).
 */
std::vector<VizLandmark> GetVizLandmarks(
    const std::vector<gtsam::Point2>& landmarks);

/**
 * @brief Convert from vector of gtsam::Point3 to vector of VizLandmarks.
 *
 * @param landmarks  Vector of GTSAM 3D points.
 * @return Vector of corresponding 3D points for visualization.
 */
std::vector<VizLandmark> GetVizLandmarks(
    const std::vector<gtsam::Point3>& landmarks);

/**
 * @brief get a color from a class id
 * 
 * @param class_id the class id
 * @return Color the color
 */
Color GetColorFromClassId(const int class_id);

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
 */
void VisualizeGtsamEstimates(
    const gtsam::Values& vals,
    const std::vector<std::vector<gtsam::Symbol>>& pose_symbols,
    const std::map<int, gtsam::Symbol>& landmark_symbols,
    const bool is3d = true, const bool animation = true,
    const size_t ms_wait = 50);

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
static void GtsamDataLoop(
    mrg::Visualizer* viz, const gtsam::Values& curr_estimate,
    const std::vector<std::vector<gtsam::Symbol>>& pose_symbols,
    const std::map<int, gtsam::Symbol>& landmark_symbols, const bool is3d,
    const bool animation, const size_t ms_wait);

}  // namespace mrg

#endif  // TONIOVIZ_GTSAMUTILS_H_
