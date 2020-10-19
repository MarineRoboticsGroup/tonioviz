/**
 * @file Visualizer.h
 * @brief Simple multi-threaded 3D graph visualization based on Pangolin.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#ifndef TONIOVIZ_VISUALIZER_H_
#define TONIOVIZ_VISUALIZER_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include <Eigen/Dense>
#include <mutex>
#include <tuple>
#include <vector>

// OpenCV includes.
#include <opencv2/opencv.hpp>

namespace mrg {

/// Trajectory consisiting of vector of Eigen-aligned 4x4 SE(3) matrices.
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
    Trajectory3;
/// 3D Pose with axes length (1st double) and line width (2nd double) for viz.
typedef std::tuple<Eigen::Matrix4d, double, double> VizPose;

/**
 * @brief Type of visualization modes available.
 */
enum class VisualizerMode { GRAPHONLY, MONO, STEREO };

/**
 * @brief Struct to hold the configuration parameters for the visualizer.
 */
struct VisualizerParams {
  float w = 1200.0f;  ///< Width of the screen [px].
  float h = 800.0f;   ///< Heigh of the screen [px].
  float f = 300.0f;   ///< Focal distance of the visualization camera [px].

  int imgwidth = 672;   ///< Width of the image to view [px].
  int imgheight = 376;  ///< Height of the image to view [px].

  VisualizerMode mode = VisualizerMode::GRAPHONLY;  ///< Type of visualizer.
};

/**
 * @class Visualizer
 * @brief Class for wrapping OpenGL and Pangoling to visualize in 3D.
 */
class Visualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Default constructor.
   * @param params  Structure with configuration parameters.
   */
  explicit Visualizer(const VisualizerParams& params = VisualizerParams());

  /**
   * @brief Default destructor.
   */
  ~Visualizer();

  /**
   * @brief Main visualization for Simple3dWorld that does all the drawing.
   */
  void RenderWorld();

  /**
   * @brief Estimates and corresponding time stamps for 3D visualization.
   * @param[in] values  System estimates.
   * @param[in] times   Corresponding timestamps.
   */
  void UpdateEstimate(const gtsam::Values& values,
                      const std::vector<double>& times);

  /**
   * @brief Add a visualization pose element.
   * @param[in] vpose   Visualization tuple with pose, axes length, and width.
   */
  void AddVizPose(const VizPose& vpose) { vposes_.push_back(vpose); }

  /**
   * @brief Add a visualization pose element.
   * @param[in] pose     3D pose of triad to visualize.
   * @param[in] length   Length of the pose axes.
   * @param[in] width    Width of the pose axes..
   */
  void AddVizPose(const Eigen::Matrix4d& pose, const double length,
                  const double width) {
    AddVizPose(std::make_tuple(pose, length, width));
  }

  /**
   * @brief Add a single image to visualize on top of the estimates.
   * @param[in] img  OpenCV image to be visualized.
   */
  void AddImage(const cv::Mat& img);

  /**
   * @brief Add both left and right images to be visualized on screen.
   * @param[in] left   Left OpenCV image to be visualized.
   * @param[in] right  Right OpenCV image to be visualized.
   */
  void AddStereo(const cv::Mat& left, const cv::Mat& right);

  /**
   * @brief Get the internal copy of the parameters used for construction.
   * @return The internal parameters structure.
   */
  inline VisualizerParams Params() const { return p_; }

 private:
  /**
   * @brief Renders a world consisting of poses and landmarks.
   * @param[in] trajectory Eigen-aligned vector of 3D poses.
   * @param[in] landmarks Vector of 3D points.
   */
  void DrawWorld(const Trajectory3& trajectory,
                 const std::vector<Eigen::Vector3d>& landmarks) const;

  /**
   * @brief Renders the trajectory as a sequence of triads.
   * @param[in] trajectory Eigen-aligned vector of 3D poses.
   */
  void DrawTrajectory(const Trajectory3& trajectory,
                      const double axesLength = 0.2) const;

  void DrawObserver() const;

  VisualizerParams p_;  ///< Internal copy of the configuration parameters.

  // Manually-modifiable variables.
  std::vector<VizPose> vposes_;  ///< Manually added poses to visualize.

  // OpenCV and image related variables.
  cv::Mat imgL_, imgR_;  ///< Left and right images.

  // GTSAM estimates.
  gtsam::Values vals_;         ///< Current system estimates.
  std::vector<double> times_;  ///< Corresponding estimate timestamps.
  Trajectory3 est_;            ///< Current state estimate trajectory.
  Trajectory3 tgt_;            ///< Current trajectory estimate for the target.

  // For safe threading.
  mutable std::mutex vizmtx_;
};

}  // namespace mrg

#endif  // TONIOVIZ_VISUALIZER_H_
