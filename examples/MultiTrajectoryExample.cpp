/**
 * @file MultiTrajectoryExample.cpp
 * @brief Quick GTSAM visualization test.
 * @author Alan Papalia, apapalia@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

// NOLINTNEXTLINE
#include <chrono>
#include <fstream>
#include <iostream>
// NOLINTNEXTLINE
#include <thread>

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
  size_t cnt = 0;
  float k = 0.03;
  while (cnt < 1000) {
    // Add a dummy pose just for funsies.
    Eigen::Matrix4d p = Eigen::Matrix4d::Identity();
    p(0, 3) = 2 * std::sin(cnt * k);                      // x
    p(1, 3) = 2 * std::sin(cnt * k) * std::cos(cnt * k);  // y
    p(2, 3) = (cnt * k);                                  // z
    viz->AddVizPose(p, 0.2, 3.0);

    p(0, 3) = 2 * std::sin(M_PI + cnt * k);                             // x
    p(1, 3) = 2 * std::sin(M_PI + cnt * k) * std::cos(M_PI + cnt * k);  // y
    p(2, 3) = (cnt * k);                                                // z
    viz->AddVizPose(p, 0.2, 3.0, 1);

    std::this_thread::sleep_for(std::chrono::nanoseconds(50000000));
    cnt++;
  }
}
