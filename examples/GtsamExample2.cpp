/**
 * @file GtsamExample2.cpp
 * @brief Slightly more advanced GTSAM visualization test to check automatic
 * visualization given values and symbols for landmarks and pose chains
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
gtsam::Values GetDummyGtsamValues(
    const std::vector<std::vector<gtsam::Symbol>> pose_symbols,
    const std::map<int, gtsam::Symbol> landmark_symbols, const bool is3d);
gtsam::Pose2 GetPose2(size_t robot, size_t time);
gtsam::Pose3 GetPose3(size_t robot, size_t time);
std::vector<gtsam::Point2> GetPoint2Vector(size_t num_landmarks);
std::vector<gtsam::Point3> GetPoint3Vector(size_t num_landmarks);

std::array<char, 12> robot_chars = {'a', 'b', 'c', 'd', 'e', 'f',
                                    'g', 'h', 'j', 'k', 'l', 'm'};
float k = 0.1;

inline char get_robot_char(int rob_id) { return robot_chars[rob_id]; }
inline gtsam::Symbol get_pose_symbol(size_t rob_id, size_t pose_id) {
  return gtsam::Symbol(get_robot_char(rob_id), pose_id);
}
inline gtsam::Symbol get_landmark_symbol(size_t land_id) {
  return gtsam::Symbol('L', land_id);
}

using LandmarkPair = std::pair<int, gtsam::Symbol>;

int main() {
  size_t num_poses = 1000;
  size_t num_landmarks = 10;
  size_t num_robots = 5;
  bool is3d = true;
  bool animation = true;
  size_t ms_wait = 10;

  // make 2d array of pose symbols
  std::vector<std::vector<gtsam::Symbol>> pose_symbols(num_robots);
  for (size_t robot = 0; robot < num_robots; robot++) {
    for (size_t time = 0; time < num_poses; time++) {
      pose_symbols[robot].push_back(get_pose_symbol(robot, time));
    }
  }

  // make map of landmark symbols
  std::map<int, gtsam::Symbol> landmark_symbols;
  LandmarkPair land_pair;
  for (size_t i = 0; i < num_landmarks; i++) {
    size_t land_id = 3 * i;
    land_pair = LandmarkPair(land_id, get_landmark_symbol(land_id));
    landmark_symbols.insert(land_pair);
  }

  // fill up trajectories and landmarks for each symbol
  gtsam::Values values;
  values = GetDummyGtsamValues(pose_symbols, landmark_symbols, is3d);

  // visualize trajectories and landmarks
  mrg::VisualizeGtsamEstimates(values, pose_symbols, landmark_symbols, is3d,
                               animation, ms_wait);
}

/* ************************************************************************** */
gtsam::Values GetDummyGtsamValues(
    const std::vector<std::vector<gtsam::Symbol>> pose_symbols,
    const std::map<int, gtsam::Symbol> landmark_symbols, const bool is3d) {
  gtsam::Values values;
  gtsam::Symbol sym;

  // fill up poses
  for (size_t robot = 0; robot < pose_symbols.size(); robot++) {
    for (size_t time = 0; time < pose_symbols[robot].size(); time++) {
      sym = pose_symbols[robot][time];
      if (is3d) {
        values.insert(sym, GetPose3(robot, time));
      } else {
        values.insert(sym, GetPose2(robot, time));
      }
    }
  }

  // fill up landmarks
  if (is3d) {
    std::vector<gtsam::Point3> landmarks =
        GetPoint3Vector(landmark_symbols.size());

    uint8_t cnt = 0;
    for (std::map<int, gtsam::Symbol>::const_iterator it =
             landmark_symbols.begin();
         it != landmark_symbols.end(); it++) {
      sym = it->second;
      values.insert(sym, landmarks[cnt]);
      cnt++;
    }
  } else {
    std::vector<gtsam::Point2> landmarks =
        GetPoint2Vector(landmark_symbols.size());

    uint8_t cnt = 0;
    for (std::map<int, gtsam::Symbol>::const_iterator it =
             landmark_symbols.begin();
         it != landmark_symbols.end(); it++) {
      sym = it->second;
      values.insert(sym, landmarks[cnt]);
      cnt++;
    }
  }

  return values;
}

gtsam::Pose2 GetPose2(size_t robot, size_t time) {
  float x = 2 * robot + 2 * std::sin(robot * M_PI + time * k);  // x
  float y = 2 * std::sin(robot * M_PI + time * k) *
            std::cos(robot * M_PI + time * k);  // y
  float rad = (time * k);                       // radians

  return gtsam::Pose2(x, y, rad);
}

gtsam::Pose3 GetPose3(size_t robot, size_t time) {
  float x = 2 * robot + 2 * std::sin(robot * M_PI + time * k);  // x
  float y = 2 * std::sin(robot * M_PI + time * k) *
            std::cos(robot * M_PI + time * k);  // y
  float z = std::sin(robot * M_PI + 2*time * k);  // z
  float yaw = (time * k);                       // yaw
  float pitch = (time * k);                     // pitch
  float roll = (time * k);                      // roll

  return gtsam::Pose3(gtsam::Rot3::Yaw(yaw), gtsam::Point3(x, y, z));
}

std::vector<gtsam::Point2> GetPoint2Vector(size_t num_landmarks) {
  // set up randomization as per links
  // https://stackoverflow.com/questions/686353/random-float-number-generation
  // https://diego.assencio.com/?index=6890b8c50169ef45b74db135063c227c
  std::random_device rnd_dev;
  std::mt19937 generator{rnd_dev()};  // Generates random integers
  std::uniform_real_distribution<> distribution{-5, 5};

  // randomly fill vectors
  std::vector<float> x_locs(num_landmarks);
  std::vector<float> y_locs(num_landmarks);
  for (size_t i = 0; i < num_landmarks; i++) {
    x_locs[i] = distribution(generator);
    y_locs[i] = distribution(generator);
  }

  // fill landmarks vector
  std::vector<gtsam::Point2> landmarks;
  for (size_t i = 0; i < num_landmarks; i++) {
    landmarks.emplace_back(gtsam::Point2(x_locs[i], y_locs[i]));
  }

  return landmarks;
}

std::vector<gtsam::Point3> GetPoint3Vector(size_t num_landmarks) {
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
  std::vector<gtsam::Point3> landmarks;
  for (size_t i = 0; i < num_landmarks; i++) {
    landmarks.emplace_back(gtsam::Point3(x_locs[i], y_locs[i], z_locs[i]));
  }

  return landmarks;
}