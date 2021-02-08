/**
 * @file GtsamUtils.cpp
 * @brief Utility functions for interacting with GTSAM objects.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include "tonioviz/GtsamUtils.h"

namespace mrg {

/**
 * TODO(alan) Implement me.
 */

void visualize_isam_estimates(gtsam::ISAM2 isam, std::string output_file) {
  // TODO (alan) finish this function

  // get value estimates and corresponding graph
  gtsam::Values curr_estimate = isam.calculateEstimate();
  gtsam::NonlinearFactorGraph graph = isam.getFactorsUnsafe();

  // write all info to g2o file
  gtsam::writeG2o(graph, curr_estimate, output_file);

  // visualize contents of file
  VisualizeG2o(output_file);

}

}  // namespace mrg
