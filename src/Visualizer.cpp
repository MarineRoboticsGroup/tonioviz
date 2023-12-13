/**
 * @file Visualizer.h
 * @brief Simple multi-threaded 3D graph visualization based on Pangolin.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include "tonioviz/Visualizer.h"

#include <Eigen/StdVector>
#include <utility>

namespace mrg {

/* *************************************************************************  */
Visualizer::Visualizer(const VisualizerParams& params) : p_(params) {
  // Make sure the images are never null pointers.
  imgL_ = cv::Mat(625, 1133, CV_8UC3, cv::Scalar(0, 0, 0));
  imgR_ = cv::Mat(625, 1133, CV_8UC3, cv::Scalar(0, 0, 0));

  // Force width and height to be evenly divisible by 4 for Pangolin. See this
  // for more details: https://github.com/stevenlovegrove/Pangolin/issues/590.
  p_.imgwidth = p_.imgwidth / 4 * 4;
  p_.imgheight = p_.imgheight / 4 * 4;
}

/* *************************************************************************  */
Visualizer::~Visualizer() {}

void Visualizer::RenderWorld() {
  std::cout << "Starting the visualization thread." << std::endl;
  pangolin::CreateWindowAndBind(_window_name, p_.w, p_.h);
  glEnable(GL_DEPTH_TEST);

  // TODO(tonioteran) Figure out what the rest of the hardcoded params are, and
  // abstract for manual configuration using an external file?

  // Define Projection and initial ModelView matrix.
  pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(
      p_.w, p_.h, p_.f, p_.f, p_.w / 2.0, p_.h / 2.0, 0.2, 1000);
  pangolin::OpenGlRenderState s_cam(
      proj,
      pangolin::ModelViewLookAt(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, pangolin::AxisZ));

  // Create the individual cameras for each view.
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0)
                              .SetHandler(new pangolin::Handler3D(s_cam));
  // Create the image viewer for either mono or stereo.
  pangolin::View& left_cam =
      pangolin::CreateDisplay().SetBounds(0.05, 0.3, 0.05, 0.5);
  pangolin::View& right_cam =
      pangolin::CreateDisplay().SetBounds(0.05, 0.3, 0.5, 0.95);

  // Toggle between drawing the origin.
  registerPangolinCallback('z', "Draw the origin and ground plane", [&]() { vis_state.show_z0 = !vis_state.show_z0; });
  // Toggle between drawing only the latest keyframe or full pose history.
  registerPangolinCallback('l', "Show only the latest pose",
                                     [&]() { vis_state.show_only_latest = !vis_state.show_only_latest; });
  // Toggle between types of keyframes
  registerPangolinCallback('k',"Toggle between Frustum or Triad keyframes",  [&]() {
    if (p_.kftype == KeyframeDrawType::kFrustum) {
      p_.kftype = KeyframeDrawType::kTriad;
    } else if (p_.kftype == KeyframeDrawType::kTriad) {
      p_.kftype = KeyframeDrawType::kFrustum;
    }
  });
  registerPangolinCallback('q', "Quit", [&]() {
    pangolin::QuitAll();
    pangolin::DestroyWindow(_window_name);
    forced_quit_ = true;
  });

  registerPangolinCallback('t', "Show trajectory",[&]() { vis_state.show_traj = !vis_state.show_traj; });
  registerPangolinCallback('m', "Show landmarks", [&]() { vis_state.show_landmark = !vis_state.show_landmark; });
  registerPangolinCallback('h', "Show help message", [&]() { vis_state.show_help = !vis_state.show_help; });
  registerPangolinCallback('r', "Show range measurements", [&]() {vis_state.show_ranges = !vis_state.show_ranges; });
  registerPangolinCallback('f', "Set view to fit all objects", [&]() {
    Eigen::Map<Eigen::Matrix2d> xy_points(xy_range_.data(), 2, 2);
    auto view_center = xy_points.rowwise().mean(); // 2x1 center of camera view
    auto view_range = (xy_points.rowwise().maxCoeff() - xy_points.rowwise().minCoeff()); // 2x1 range of camera view
    auto z_w = view_range(1) * p_.f / p_.w; // Since we are using X-up, we need to use the width to capture all of y
    auto z_h  = view_range(0) * p_.f / p_.h; // X-up, need height to capture all of x

    auto z = std::max(z_w, z_h) * 1.1; // Add a buffer around the bounding rectangle

    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(view_center(0),  view_center(1), z, view_center(0), view_center(1), 0.0, pangolin::AxisX));
  });
  // Manage the size of the points.
  glPointSize(3.5);  // Default is 1.
  // Useful identity.
  Eigen::Matrix4d I_4x4 = Eigen::Matrix4d::Identity();

  // Deal with the images.
  const int width = p_.imgwidth;    // 672;   // 1133;
  const int height = p_.imgheight;  // 376;  // 625;
  pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                   GL_UNSIGNED_BYTE);

  while (!pangolin::ShouldQuit()) {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ------------
    // -- 2D plots.

    // -----------
    // -- 3D view.
    d_cam.Activate(s_cam);
    // Background color.
    glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
    // Default line width.
    glLineWidth(1.0);

    if (vis_state.show_traj) {
      for (uint i = 0; i < pose_vectors_.size(); i++) {
        DrawTrajectory(pose_vectors_[i]);
      }
    }
    if (vis_state.show_landmark) {
      DrawLandmarks(landmarks_);
    }

    if (vis_state.show_ranges){
      DrawRanges(ranges_, p_.range_color);
    }

    s_cam.Apply();
    glColor3f(1.0, 1.0, 1.0);
    if (vis_state.show_z0) {
      auto furthest_element = xy_range_.rowwise().norm().maxCoeff();
      pangolin::glDraw_z0(1.0, std::ceil(furthest_element));

      glLineWidth(7.5);
      pangolin::glDrawAxis(I_4x4, 0.11);
    }

    DrawHelp();

    glLineWidth(1.0);

    // ----------------
    // -- Images.
    if (p_.mode == VisualizerMode::MONO || p_.mode == VisualizerMode::STEREO) {
      left_cam.Activate();
      glColor3f(1.0, 1.0, 1.0);

      vizmtx_.lock();
      imageTexture.Upload(imgL_.data, GL_BGR, GL_UNSIGNED_BYTE);
      vizmtx_.unlock();
      imageTexture.RenderToViewport();

      if (p_.mode == VisualizerMode::STEREO) {
        right_cam.Activate();
        glColor3f(1.0, 1.0, 1.0);

        vizmtx_.lock();
        imageTexture.Upload(imgR_.data, GL_BGR, GL_UNSIGNED_BYTE);
        vizmtx_.unlock();
        imageTexture.RenderToViewport();
      }
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
}

/* ************************************************************************** */
void Visualizer::AddVizPoses(const std::vector<VizPose>& vposes, int traj_ind) {
  vizmtx_.lock();
  while (static_cast<int>(pose_vectors_.size()) <= traj_ind) pose_vectors_.emplace_back();
  for (const auto& vpose : vposes) pose_vectors_[traj_ind].push_back(vpose);
  num_poses += static_cast<int>(vposes.size());
  updateXYRange();
  vizmtx_.unlock();
}

/* ************************************************************************** */
void Visualizer::AddVizPose(const Eigen::Matrix4d& pose, const double length,
                            const double width, int traj_ind) {
  AddVizPose(std::make_tuple(pose, length, width), traj_ind);
}

/* ************************************************************************** */
void Visualizer::AddVizPoses(const Trajectory3& poses, const double length,
                             const double width, int traj_ind) {
  for (const Eigen::Matrix4d& pose : poses) {
    AddVizPose(std::make_tuple(pose, length, width), traj_ind);
  }
}

/* ************************************************************************** */
void Visualizer::AddVizLandmarks(const std::vector<VizLandmark>& landmarks) {
  vizmtx_.lock();
  for (const auto& vl : landmarks) {
    landmarks_.push_back(ColorLandmark{vl, p_.landmark_color});
  }
  updateXYRange();
  vizmtx_.unlock();
}

/* ************************************************************************** */
void Visualizer::DrawTrajectory(const Trajectory3& trajectory,
                                const double axesLength) const {
  std::vector<Eigen::Vector3d> positions;

  // Draw all poses.
  for (const Eigen::Matrix4d& p : trajectory) {
    positions.push_back(p.block<3, 1>(0, 3));
  }
  glLineWidth(3.0);
  if (trajectory.size()) pangolin::glDrawAxis(trajectory.back(), axesLength);

  // Draw a line connecting all poses.
  glColor4f(0.7, 0.7, 0.7, 0.1);
  glLineWidth(3.0);
  pangolin::glDrawLineStrip(positions);
  glLineWidth(1.0);
  glColor3f(1.0, 1.0, 1.0);
}

/* *************************************************************************  */
void Visualizer::DrawTrajectory(const std::vector<VizPose>& trajectory) const {
  std::vector<Eigen::Vector3d> positions;

  // Draw all keyframes and get all positions.
  glColor3f(0.0, 0.0, 0.4);
  for (const VizPose& vp : trajectory) {
    if (!vis_state.show_only_latest || vp == trajectory.back()) {
      glLineWidth(std::get<2>(vp));
      if (p_.kftype == KeyframeDrawType::kFrustum) {
        Eigen::Matrix4d Twf = std::get<0>(vp) * T_frustum_;
        pangolin::glDrawFrustum(K_frustum_, frustum_w_, frustum_h_, Twf,
                                p_.frustum_scale);
      } else if (p_.kftype == KeyframeDrawType::kTriad) {
        pangolin::glDrawAxis(std::get<0>(vp), std::get<1>(vp));
      }
      glLineWidth(1.0);
    }
    positions.emplace_back(std::get<0>(vp).block<3, 1>(0, 3));
  }

  // Draw a line connecting all poses.
  glColor4f(0.7, 0.7, 0.7, 0.1);
  glLineWidth(3.0);
  pangolin::glDrawLineStrip(positions);
  glLineWidth(1.0);
  glColor3f(1.0, 1.0, 1.0);
}

/* *************************************************************************  */
void Visualizer::AddImage(const cv::Mat& img) {
  cv::Mat img_short;
  img.convertTo(img_short, CV_8UC3);
  // Ensure image is truncated to param height, width.
  img_short = img_short.rowRange(0, p_.imgheight).colRange(0, p_.imgwidth);
  cv::flip(img_short.clone(), imgL_, 0);
}

/* *************************************************************************  */
void Visualizer::AddStereo(const cv::Mat& left, const cv::Mat& right) {
  cv::Mat left_short, right_short;
  left.convertTo(left_short, CV_8UC3);  // Not even sure if this is necessary.
  right.convertTo(right_short, CV_8UC3);

  // Ensure left/right images are truncated to param height, width.
  left_short = left_short.rowRange(0, p_.imgheight).colRange(0, p_.imgwidth);
  right_short = right_short.rowRange(0, p_.imgheight).colRange(0, p_.imgwidth);

  vizmtx_.lock();
  cv::flip(left_short.clone(), imgL_, 0);
  cv::flip(right_short.clone(), imgR_, 0);
  vizmtx_.unlock();
}

void Visualizer::DrawHelp() const {
  // Print all keybinds with descriptions onto the pangolin canvas
  glColor3f(0, 0, 0);
  glLineWidth(1.0);

  // Built off of Tonio's code in SESyncVisualizer.cpp
  GLboolean gl_blend_enabled;
  glGetBooleanv(GL_BLEND, &gl_blend_enabled);

  // Ensure that blending is enabled for rendering text.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  if (vis_state.show_help){
    auto height = 10.0f;
    for (auto & keybind : keybinds_) {
        pangolin::GlFont::I().Text(std::string{keybind.first + std::string(": ") + keybind.second}).DrawWindow(10, height);
        height += 20;
    }
  } else {
    pangolin::GlFont::I()
        .Text(std::string{"Press 'h' to show help message"}).DrawWindow(10, 10);
  }
  // Restore previous value.
  if (!gl_blend_enabled) glDisable(GL_BLEND);

  glLineWidth(1.0);
}

void Visualizer::registerPangolinCallback(char key, std::string description,
                                          std::function<void(void)> callback) {
  if (keybinds_.find(key) != keybinds_.end()) {
    std::cout << "Keybind for " << key << " already exists. Overwriting." << std::endl;
  }
  keybinds_[key] = std::move(description);
  pangolin::RegisterKeyPressCallback(key, std::move(callback));
}

/**
 * @brief Get the bounding rectangle of poses and landmarks in the xy plane
 * @return x_min, y_min, x_max, y_max
 */
void Visualizer::updateXYRange() {
  auto num_points = landmarks_.size() + num_poses;
  if (num_points == 0) {
    return;
  }

  Eigen::MatrixXd points(num_points, 2);

  auto row_idx{0};
  for (auto& landmark : landmarks_){
    points.row(row_idx) << landmark.lmPos.block(0, 0, 2, 1).transpose();
    row_idx++;
  }
  for (auto& pose_vec: pose_vectors_) {
    for (auto& pose : pose_vec) {
      points.row(row_idx) << std::get<0>(pose).block(0, 3, 2, 1).transpose();
      row_idx++;
    }
  }

  xy_range_.block(0, 0, 2, 1) = points.colwise().minCoeff().transpose();
  xy_range_.block(2, 0, 2, 1) = points.colwise().maxCoeff().transpose();
}

}  // namespace mrg
