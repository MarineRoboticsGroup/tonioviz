/**
 * @file GtsamUtils.cpp
 * @brief Utility functions for interacting with GTSAM objects.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#ifndef TONIOVIZ_GTSAMUTILS_H_
#define TONIOVIZ_GTSAMUTILS_H_

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>

#include <string>

#include "tonioviz/DataUtils.h"

namespace mrg {

/**
 * TODO(alan) Implement me.
 */

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
