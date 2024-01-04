/**
 * @file SimpleCheckout.cpp
 * @brief Test quick visualization properties.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

// NOLINTNEXTLINE
#include <chrono>
#include <fstream>
#include <iostream>
// NOLINTNEXTLINE
#include <thread>

#include "tonioviz/DataUtils.h"
#include "tonioviz/Visualizer.h"

// Forwards declarations.
void DataPlaybackLoop(const std::vector<cv::Mat> &imgs, mrg::Visualizer *viz);

int main() {
  // Load a visual dataset.
  mrg::ImageDataset ds;
  // ds.path = "/home/tonio/data/maxmixtures/sonar_horizontal_whoi_run1/";
  ds.path = "/Users/tonio/Downloads/sonar_horizontal_whoi_run1/";
  ds.filename = "sonar_horizontal";
  ds.extension = "jpg";
  ds.start = 0;
  ds.end = 500;
  // ds.end = 2497;
  ds.zeropad = true;
  ds.digits = 4;

  std::vector<cv::Mat> imgs;
  mrg::LoadImages(ds, &imgs);
  std::cout << "Num of imgs loaded: " << imgs.size() << std::endl;

  // Create a sample visualizer object.
  mrg::VisualizerParams params;
  params.f = 600;
  // params.mode = mrg::VisualizerMode::GRAPHONLY;
  // params.mode = mrg::VisualizerMode::MONO;
  params.mode = mrg::VisualizerMode::STEREO;
  // params.onlylatest = true;
  // params.kftype = mrg::KeyframeDrawType::kTriad;
  params.imgwidth1 = 1133;
  params.imgheight1 = 625;
  params.imgwidth2 = 1133;
  params.imgheight2 = 625;
  mrg::Visualizer viz{params};

  // Add an image to the visualizer.
  if (imgs.size() > 0) viz.AddImage(imgs[0]);

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

  // Dispatch data playback loop and enter rendering loop.
  std::thread data_thread(DataPlaybackLoop, imgs, &viz);
  viz.RenderWorld();
  data_thread.join();  // Wait until both threads are finished.

  return 0;
}

/* ************************************************************************** */
void DataPlaybackLoop(const std::vector<cv::Mat> &imgs, mrg::Visualizer *viz) {
  size_t counter = 0;
  mrg::VisualizerParams params = viz->Params();
  for (const cv::Mat &img : imgs) {
    if (params.mode == mrg::VisualizerMode::MONO) {
      viz->AddImage(img);
    } else if (params.mode == mrg::VisualizerMode::STEREO) {
      viz->AddStereo(img, img);
    }

    // Add a dummy pose just for funsies.
    Eigen::Matrix4d p = Eigen::Matrix4d::Identity();
    p(0, 3) = 2.0 + (counter * 0.1);
    p(1, 3) = std::cos(counter * 0.01);
    p(2, 3) = std::sin(counter * 0.01);
    viz->AddVizPose(p, 0.2, 3.0);

    std::this_thread::sleep_for(std::chrono::nanoseconds(100000000));
    counter++;
  }
}
