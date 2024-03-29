/**
 * @file GtsamExample1.cpp
 * @brief Quick GTSAM visualization test to check use of poses from GTSAM. Uses
 * conversion to convert all GTSAM objects to normal Viz objects
 * @author Tonio Teran, teran@mit.edu
 * @author Alan Papalia, apapalia@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <chrono>  // NOLINT [build/c++11]
#include <fstream>
#include <iostream>
#include <thread>  // NOLINT [build/c++11]

#include "tonioviz/GtsamUtils.h"
#include "tonioviz/Visualizer.h"

// Forwards declarations.
void DataPlaybackLoop(mrg::Visualizer *viz);
gtsam::Values GetDummyGtsamValues(const size_t size);
thread_local unsigned int seed = time(NULL);

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

double GetRandDouble(double low, double high) {
  return (rand_r(&seed) % 100) / 100.0 * (high - low) + low;
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

    const double x = GetRandDouble(-10, 10);
    const double y = GetRandDouble(-10, 10);
    const double r = GetRandDouble(1, 10);

    mrg::Range c = {x, y, r};
    viz->AddRangeMeasurement(c);

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
