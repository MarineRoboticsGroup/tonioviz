/**
 * @file DataUtils.cpp
 * @brief Utility functions for interacting with the raw datasets.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#ifndef TONIOVIZ_DATAUTILS_H_
#define TONIOVIZ_DATAUTILS_H_

#include <chrono>  //NOLINT
#include <opencv4/opencv2/opencv.hpp>
#include <string>  //NOLINT [build/include_order]
#include <vector>  //NOLINT [build/include_order]

namespace mrg {

/**
 * @brief Bundled properties for loading a visual dataset.
 */
struct ImageDataset {
  std::string path;       ///< Path to image location.
  std::string filename;   ///< File name without extension.
  std::string extension;  ///< Images extension.

  size_t start;  ///< Starting image index.
  size_t end;    ///< Last image index.

  bool zeropad;   ///< Whether zero padding is needed for filenames.
  size_t digits;  ///< If padding is needed, specify the number of digits.
};

/**
 * @brief Reads a configuration file and loads all stereo images from a dataset.
 * @param[in]  ds  Dataset configuration.
 * @param[out] images  Vector of OpenCV images to hold loaded data.
 */
void LoadImages(const ImageDataset& ds, std::vector<cv::Mat>* images);

void VisualizeG2o(const std::string g2o_file);

}  // namespace mrg

#endif  // TONIOVIZ_DATAUTILS_H_
