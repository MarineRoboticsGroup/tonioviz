/**
 * @file DataUtils.cpp
 * @brief Utility functions for interacting with the raw datasets.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include "tonioviz/DataUtils.h"

#include <fstream>
#include <string>

namespace mrg {

/* ************************************************************************** */
void LoadImages(const ImageDataset& ds, std::vector<cv::Mat>* images) {
  std::string prefix = ds.path + ds.filename;
  std::string end = "." + ds.extension;
  std::string padnum;
  size_t num_imgs = ds.end - ds.start;

  // Allocate memory.
  images->clear();
  images->reserve(num_imgs);

  for (size_t i = ds.start; i < ds.end; i++) {
    std::string numstr = std::to_string(i);
    padnum = ds.zeropad ? std::string(ds.digits - numstr.length(), '0') + numstr
                        : numstr;
    std::string name = prefix + padnum + end;
    std::cout << "Reading image: " << name << std::endl;
    cv::Mat im = cv::imread(name, cv::IMREAD_COLOR);
    images->push_back(im);
  }
}

}  // namespace mrg
