/**
 * @file GtsamExample1.cpp
 * @brief Quick GTSAM visualization test to check use of poses from GTSAM
 * @author Tonio Teran, teran@mit.edu
 * @author Alan Papalia, apapalia@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

// NOLINTNEXTLINE
#include <chrono>
#include <fstream>
#include <iostream>
// NOLINTNEXTLINE
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <thread>

#include "tonioviz/GtsamUtils.h"
#include "tonioviz/Visualizer.h"

// Forwards declarations.
void DataPlaybackLoop(mrg::Visualizer *viz);
gtsam::Values GetDummyGtsamValues(const size_t size);

int main() {
  // Create a sample visualizer object.
  mrg::VisualizerParams params;
  params.f = 600;
  params.mode = mrg::VisualizerMode::GRAPHONLY;
  params.kftype = mrg::KeyframeDrawType::kTriad;
  mrg::Visualizer viz{params};

  // Dispatch data playback loop and enter rendering loop.
  std::thread data_thread(DataPlaybackLoop, &viz);
  viz.RenderWorld();
  data_thread.join();  // Wait until both threads are finished.

  return 0;
}

/* ************************************************************************** */
void DataPlaybackLoop(mrg::Visualizer *viz) {
  size_t counter = 0;
  size_t num_poses = 1000;

  gtsam::Values values;
  while (counter < num_poses) {
    values = GetDummyGtsamValues(counter + 1);

    viz->Clear();  // Make sure to clear the visualizer first!!!
    viz->AddVizPoses(mrg::GetVizPoses(values));  // Add straight from gtsam.

    std::this_thread::sleep_for(std::chrono::nanoseconds(50000000));
    counter++;
  }
}

/* ************************************************************************** */
gtsam::Values GetDummyGtsamValues(const size_t size) {
  gtsam::Values values;

  for (int i = 0; i < size; i++) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    double radius = 0.05 * i;
    double angle = 0.1 * i;
    pose(0, 3) = radius * std::cos(angle);
    pose(1, 3) = radius * std::sin(angle);

    values.insert(gtsam::Symbol('x', i), gtsam::Pose3(pose));
  }

  return values;
}
