/**
 * @file Visualizer.h
 * @brief Simple multi-threaded 3D graph visualization based on Pangolin.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include "tonioviz/Visualizer.h"

#include <Eigen/StdVector>

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

  // Initialize pose vector with empty vector of poses.
  pose_vectors_.emplace_back(std::vector<VizPose>());
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

  // Real-time toggles using key presses.
  bool show_z0 = true;
  // Toggle between drawing the origin.
  pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });
  // Toggle between drawing only the latest keyframe or full pose history.
  pangolin::RegisterKeyPressCallback('l',
                                     [&]() { p_.onlylatest = !p_.onlylatest; });
  // Toggle between types of keyframes
  pangolin::RegisterKeyPressCallback('k', [&]() {
    if (p_.kftype == KeyframeDrawType::kFrustum) {
      p_.kftype = KeyframeDrawType::kTriad;
    } else if (p_.kftype == KeyframeDrawType::kTriad) {
      p_.kftype = KeyframeDrawType::kFrustum;
    }
  });
  pangolin::RegisterKeyPressCallback('q', [&]() {
    pangolin::QuitAll();
    pangolin::DestroyWindow(_window_name);
    forced_quit_ = true;
  });

  bool show_manual = true;
  pangolin::RegisterKeyPressCallback('m',
                                     [&]() { show_manual = !show_manual; });

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

    if (show_manual) {
      for (uint i = 0; i < pose_vectors_.size(); i++) {
        DrawTrajectory(pose_vectors_[i]);
      }
      DrawLandmarks(landmarks_, p_.landmark_color);
    }

    DrawRanges(ranges_, p_.range_color);

    s_cam.Apply();
    glColor3f(1.0, 1.0, 1.0);
    if (show_z0) pangolin::glDraw_z0(1.0, 20);
    glLineWidth(7.5);
    if (show_z0) pangolin::glDrawAxis(I_4x4, 0.11);
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
void Visualizer::AddVizPoses(const std::vector<VizPose>& vposes) {
  for (const auto& vpose : vposes) pose_vectors_[0].push_back(vpose);
}

/* ************************************************************************** */
void Visualizer::AddVizPoses(const std::vector<VizPose>& vposes, int traj_ind) {
  for (const auto& vpose : vposes) pose_vectors_[traj_ind].push_back(vpose);
}

/* ************************************************************************** */
void Visualizer::AddVizPose(const Eigen::Matrix4d& pose, const double length,
                            const double width) {
  AddVizPose(std::make_tuple(pose, length, width));
}

/* ************************************************************************** */
void Visualizer::AddVizPose(const Eigen::Matrix4d& pose, const double length,
                            const double width, int traj_ind) {
  AddVizPose(std::make_tuple(pose, length, width), traj_ind);
}

/* ************************************************************************** */
void Visualizer::AddVizPoses(const Trajectory3& poses, const double length,
                             const double width) {
  for (const Eigen::Matrix4d& pose : poses) {
    AddVizPose(std::make_tuple(pose, length, width));
  }
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
  for (const auto& vl : landmarks) landmarks_.push_back(vl);
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
    if (!p_.onlylatest) {
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
    positions.push_back(std::get<0>(vp).block<3, 1>(0, 3));
  }

  // Draw only the most recent keyframe.
  if (p_.onlylatest && trajectory.size()) {
    VizPose latest = trajectory.back();
    glLineWidth(std::get<2>(latest));
    if (p_.kftype == KeyframeDrawType::kFrustum) {
      Eigen::Matrix4d Twf = std::get<0>(latest) * T_frustum_;
      pangolin::glDrawFrustum(K_frustum_, frustum_w_, frustum_h_, Twf,
                              p_.frustum_scale);
    } else if (p_.kftype == KeyframeDrawType::kTriad) {
      pangolin::glDrawAxis(std::get<0>(latest), std::get<1>(latest));
    }
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

}  // namespace mrg
