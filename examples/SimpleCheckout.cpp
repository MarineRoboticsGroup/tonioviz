/**
 * @file SimpleCheckout.cpp
 * @brief Test quick visualization properties.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include <fstream>
#include <iostream>

#include "tonioviz/Visualizer.h"

int main() {
  // Create a sample visualizer object.
  mrg::VisualizerParams params;
  params.f = 600;
  mrg::Visualizer viz{params};  // NOTE: default configuration.

  // Create a couple dummy poses.
  Eigen::Matrix4d p0 = Eigen::Matrix4d::Identity(),
                  p1 = Eigen::Matrix4d::Identity(),
                  p2 = Eigen::Matrix4d::Identity();
  p1(0, 3) = 1.0;
  p2(0, 3) = 2.0;

  // Add dummy poses to visualizer.
  viz.AddVizPose(p0, 0.1, 2.0);  // Params: triad length, triad width.
  viz.AddVizPose(p1, 0.2, 3.0);  // Params: triad length, triad width.
  viz.AddVizPose(p2, 0.3, 4.0);  // Params: triad length, triad width.

  // Enter rendering loop.
  viz.RenderWorld();

  return 0;
}
