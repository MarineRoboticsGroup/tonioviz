/**
 * @file GtsamUtils.cpp
 * @brief Utility functions for interacting with GTSAM objects.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#ifndef TONIOVIZ_GTSAMUTILS_H_
#define TONIOVIZ_GTSAMUTILS_H_

#include <string>
#include <vector>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "tonioviz/DataUtils.h"
#include "tonioviz/Visualizer.h"

namespace mrg {

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
 * @brief Writes current values of variables inside isam object to a g2o file
 * and then visualizes the contents of the g2o file
 *
 * @param isam the isam object to visualize all current estimates of
 * @param output_file the filepath where the saved g2o representation of the
 * estimates will be kept
 */
void visualize_isam_estimates(gtsam::ISAM2 isam, std::string output_file);

}  // namespace mrg

#endif  // TONIOVIZ_GTSAMUTILS_H_
