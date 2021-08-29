/**
 * @file Visualizer.h
 * @brief Simple multi-threaded 3D graph visualization based on Pangolin.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#ifndef TONIOVIZ_VISUALIZER_H_
#define TONIOVIZ_VISUALIZER_H_

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include <Eigen/Dense>
// NOLINTNEXTLINE
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

// OpenCV includes.
#include <opencv2/opencv.hpp>

namespace mrg {

/// Trajectory consisting of vector of Eigen-aligned 4x4 SE(3) matrices.
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
    Trajectory3;
/// 3D Pose with axes length (1st double) and line width (2nd double) for viz.
typedef std::tuple<Eigen::Matrix4d, double, double> VizPose;

/// 3D Position of landmark
typedef Eigen::Vector3d VizLandmark;

/**
 * @brief Type of visualization modes available.
 */
enum class VisualizerMode { GRAPHONLY, MONO, STEREO };

/**
 * @brief Type of keyframe representation for drawing.
 */
enum class KeyframeDrawType { kFrustum, kTriad, kPoint };
enum class LandmarkDrawType { kCross, kPoint };

/**
 * @brief Struct to hold circles with radius `r` for drawing at (`x`,`y`).
 */
struct Circle {
  double x, y, r;
};

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
  LandmarkDrawType landtype = LandmarkDrawType::kPoint;  ///< Landmark type.
  bool onlylatest = false;     ///< Draw only the most recent keyframe.
  double frustum_scale = 0.1;  ///< Size of frustum [m].
};

/**
 * @class Visualizer
 * @brief Class for wrapping OpenGL and Pangoling to visualize in 3D. Will
 * default to single trajectory visualization but allows for visualizing
 * multiple trajectories.
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

  inline void AddRangeMeasurement(const Circle c) { circles_.push_back(c); }

  /**
   * @brief Add a visualization pose element.
   * @param[in] vpose   Visualization tuple with pose, axes length, and width.
   */
  inline void AddVizPose(const VizPose& vpose) {
    pose_vectors_[0].push_back(vpose);
  }

  /**
   * @brief Add a visualization pose element. For multiple trajectories.
   * @param[in] vpose   Visualization tuple with pose, axes length, and width.
   */
  inline void AddVizPose(const VizPose& vpose, uint traj_ind) {
    while (pose_vectors_.size() <= traj_ind) {
      pose_vectors_.emplace_back(std::vector<VizPose>());
    }
    pose_vectors_[traj_ind].push_back(vpose);
  }

  /**
   * @brief Add a visualization pose element; overload with individual elements.
   * @param[in] pose     3D pose of triad to visualize.
   * @param[in] length   Length of the pose axes.
   * @param[in] width    Width of the pose axes..
   */
  void AddVizPose(const Eigen::Matrix4d& pose, const double length,
                  const double width);

  /**
   * @brief Add a visualization pose element; overload with individual elements.
   * For multiple trajectories.
   * @param[in] pose     3D pose of triad to visualize.
   * @param[in] length   Length of the pose axes.
   * @param[in] width    Width of the pose axes..
   */
  void AddVizPose(const Eigen::Matrix4d& pose, const double length,
                  const double width, int traj_ind);

  /**
   * @brief Same as above, but for multiple poses at a time..
   * @param[in] vposes   Vector of visualization poses.
   */
  void AddVizPoses(const std::vector<VizPose>& vposes);

  /**
   * @brief Same as above, but for multiple poses at a time. For multiple
   * trajectories.
   * @param[in] vposes   Vector of visualization poses.
   */
  void AddVizPoses(const std::vector<VizPose>& vposes, int traj_ind);

  /**
   * @brief Same as above, but for multiple poses at the same time..
   * @param[in] poses    Vector of 3D poses to visualize.
   * @param[in] length   Length of the pose axes.
   * @param[in] width    Width of the pose axes.
   */
  void AddVizPoses(const Trajectory3& poses, const double length,
                   const double width);

  /**
   * @brief Same as above, but for multiple poses at the same time. For multiple
   * trajectories.
   * @param[in] poses    Vector of 3D poses to visualize.
   * @param[in] length   Length of the pose axes.
   * @param[in] width    Width of the pose axes.
   */
  void AddVizPoses(const Trajectory3& poses, const double length,
                   const double width, int traj_ind);

  /**
   * @brief Adds a landmark to be visualized
   *
   * @param vl landmark
   */
  inline void AddVizLandmark(const VizLandmark& vl) {
    landmarks_.push_back(vl);
  }

  /**
   * @brief Adds multiple landmarks to be visualized
   *
   * @param landmarks vector of landmarks to visualize
   */
  void AddVizLandmarks(const std::vector<VizLandmark>& landmarks);

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

  /**
   * @brief Clears all the stored visualized poses.
   */
  inline void Clear() {
    for (uint i = 0; i < pose_vectors_.size(); i++) {
      pose_vectors_[i].clear();
    }
  }

  /**
   * @brief Allows to check if the visualizer has been told to force quit
   */
  inline bool HasForcedQuit() { return forced_quit_; }

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

  inline void DrawLandmarks(const std::vector<VizLandmark>& landmarks) const {
    // Draw all landmarks
    glColor3f(1.0f, 0.0, 0.0);
    double rad = 0.25;
    if (p_.landtype == LandmarkDrawType::kCross) {
      for (const VizLandmark& vl : landmarks) {
        pangolin::glDrawCross(vl, rad);
      }
    } else if (p_.landtype == LandmarkDrawType::kPoint) {
      pangolin::glDrawPoints(landmarks);
    } else {
      std::cerr << "Attempted landmark visualization is not supported"
                << std::endl;
      assert(false);
    }

    glLineWidth(1.0);
  }

  inline void DrawLandmarks() const { DrawLandmarks(landmarks_); }

  inline void DrawCircles() const { DrawCircles(circles_); }

  inline void DrawCircles(std::vector<Circle> circles) const {
    for (const Circle c : circles) {
      DrawCircle(c);
    }
  }

  inline void DrawCircle(Circle c) const {
    pangolin::glDrawCirclePerimeter(c.x, c.y, c.r);
  }

  VisualizerParams p_;  ///< Internal copy of the configuration parameters.
  const std::string _window_name =
      "mrg official viewer";  // name of the visualizer window
  bool forced_quit_ = false;

  // Manually-modifiable variables.
  std::vector<std::vector<VizPose>>
      pose_vectors_;  ///< 2D vector of poses to represent multiple trajectories
  std::vector<VizLandmark> landmarks_;
  std::vector<Circle> circles_;

  // OpenCV and image related variables.
  cv::Mat imgL_, imgR_;  ///< Left and right images.

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

  // Frustum shape (width and height).
  const int frustum_w_ = 2, frustum_h_ = 1;
  // Parameter matrix for the frustum shape, of the following form:
  //
  //         [ fu      u0 ]
  //  Kinv = [     fv  v0 ]
  //         [          1 ]
  //
  // We're assuming a frustum of width = 2 and height = 1, yielding u0 = -1 and
  // v0 = -0.5, with fu = fv = 1.
  // clang-format off
  const Eigen::Matrix3d K_frustum_ =
       (Eigen::Matrix3d() << 1, 0, -1.0,
                             0, 1, -0.5,
                             0, 0,  1.0).finished();
  // clang-format on
};

}  // namespace mrg

#endif  // TONIOVIZ_VISUALIZER_H_
