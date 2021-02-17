/**
 * @file LandmarkExample.cpp
 * @brief Example of visualizing landmarks (points) in tonioviz
 * @author Alan Papalia, apapalia@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
 */

// NOLINTNEXTLINE
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
// NOLINTNEXTLINE
#include <algorithm>
#include <functional>
#include <iterator>
#include <random>
#include <vector>

#include "tonioviz/Visualizer.h"

// Forwards declarations.
void DataPlaybackLoop(mrg::Visualizer *viz);

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
  size_t num_landmarks = 20;
  const double rad = 0.2;
  const double width = 0.2;

  // set up randomization as per links
  // https://stackoverflow.com/questions/686353/random-float-number-generation
  // https://diego.assencio.com/?index=6890b8c50169ef45b74db135063c227c
  std::random_device rnd_dev;
  std::mt19937 generator{rnd_dev()};  // Generates random integers
  std::uniform_real_distribution<> distribution{-5, 5};

  // randomly fill vectors
  std::vector<float> x_locs(num_landmarks);
  std::vector<float> y_locs(num_landmarks);
  std::vector<float> z_locs(num_landmarks);
  for (size_t i = 0; i < num_landmarks; i++) {
    x_locs[i] = distribution(generator);
    y_locs[i] = distribution(generator);
    z_locs[i] = distribution(generator);
  }

  // fill landmarks vector
  std::vector<mrg::VizLandmark> landmarks;
  for (size_t i = 0; i < num_landmarks; i++) {
    landmarks.emplace_back(mrg::VizLandmark(x_locs[i], y_locs[i], z_locs[i]));
  }

  // add landmarks to visualizer
  viz->AddVizLandmarks(landmarks);

  //   size_t cnt = 0;
  //   while (cnt < num_landmarks) {
  // std::this_thread::sleep_for(std::chrono::nanoseconds(50000000));
  // cnt++;
  //   }
}
