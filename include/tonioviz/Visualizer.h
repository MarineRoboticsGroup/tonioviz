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
// NOLINTNEXTLINE
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
// typedef std::vector<VizPose, Eigen::aligned_allocator<Eigen::Matrix4d>>
//     VizPoseVec;

/**
 * @brief Type of visualization modes available.
 */
enum class VisualizerMode { GRAPHONLY, MONO, STEREO };

/**
 * @brief Type of keyframe representation for drawing.
 */
enum class KeyframeDrawType { kFrustum, kTriad, kPoint };

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

  KeyframeDrawType kftype = KeyframeDrawType::kFrustum;  ///< Keyframe type.
  bool onlylatest = true;  ///< Draw only the most recent keyframe.
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
   * @brief Add a visualization pose element.
   * @param[in] vpose   Visualization tuple with pose, axes length, and width.
   */
  inline void AddVizPose(const VizPose& vpose) { vposes_.push_back(vpose); }

  /**
   * @brief Add a visualization pose element; overload with individual elements.
   * @param[in] pose     3D pose of triad to visualize.
   * @param[in] length   Length of the pose axes.
   * @param[in] width    Width of the pose axes..
   */
  void AddVizPose(const Eigen::Matrix4d& pose, const double length,
                  const double width);

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
   * @brief Renders the trajectory as a sequence of poses.
   * @param[in] trajectory  Eigen-aligned vector of 3D poses.
   */
  void DrawTrajectory(const Trajectory3& trajectory,
                      const double axesLength = 0.2) const;

  /**
   * @brief Overload to render a trajectory of pose tuples.
   * @param[in] trajectory  Eigen-aligned vector of visualization pose tuples.
   */
  void DrawTrajectory(const std::vector<VizPose>& trajectory) const;

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

  // Frustum rotation to align with +X axis (instead of +Z). Just a rotation of
  // 90deg about +Z, followed by a 90deg rotation about +Y.
  // clang-format off
  const Eigen::Matrix4d T_frustum_ =
      (Eigen::Matrix4d() << 0, 0, 1, 0,
                            1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 0, 1).finished();
  // clang-format on
};

}  // namespace mrg

#endif  // TONIOVIZ_VISUALIZER_H_
