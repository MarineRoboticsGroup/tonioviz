/**
 * @file GtsamUtils.cpp
 * @brief Utility functions for interacting with GTSAM objects.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include "tonioviz/GtsamUtils.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

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

std::vector<VizPose> GetVizPoses(const gtsam::Values& values,
                                 const double length, const double width,
                                 const unsigned char c, const bool is3d) {
  std::vector<VizPose> vposes;
  vposes.reserve(values.size());

  size_t counter = 0;
  while (values.exists(gtsam::Symbol(c, counter))) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

    if (is3d) {  // gtsam::Pose3
      gtsam::Pose3 pose3 = values.at<gtsam::Pose3>(gtsam::Symbol(c, counter));
      pose = pose3.matrix();
    } else {  // gtsam::Pose2
      gtsam::Pose2 pose2 = values.at<gtsam::Pose2>(gtsam::Symbol(c, counter));
      Eigen::Matrix3d pose2mat = pose2.matrix();
      // Set rotation part and translation part.
      pose.block(0, 0, 2, 2) = pose2mat.block(0, 0, 2, 2);
      pose.block(0, 3, 2, 1) = pose2mat.block(0, 2, 2, 1);
    }

    vposes.push_back(std::make_tuple(pose, length, width));
    counter++;
  }

  return vposes;
}

}  // namespace mrg
