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
}

/* *************************************************************************  */
Visualizer::~Visualizer() {}

void Visualizer::RenderWorld() {
  std::cout << "Starting the visualization thread." << std::endl;
  pangolin::CreateWindowAndBind("spheres-vertigo viewer", p_.w, p_.h);
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
  pangolin::View& images_cam =
      pangolin::CreateDisplay().SetBounds(0.05, 0.3, 0.05, 0.5);

  // Real-time toggles using key presses.
  bool show_z0 = true;
  pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });

  bool show_gtsam = true;
  pangolin::RegisterKeyPressCallback('g', [&]() { show_gtsam = !show_gtsam; });

  bool show_manual = true;
  pangolin::RegisterKeyPressCallback('m',
                                     [&]() { show_manual = !show_manual; });

  // Manage the size of the points.
  glPointSize(3.5);  // Default is 1.
  // Useful identity.
  Eigen::Matrix4d I_4x4 = Eigen::Matrix4d::Identity();

  // Deal with the images.
  const int width = 1133;
  const int height = 625;
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

    if (show_gtsam) {
      DrawObserver();
      DrawTarget();
    }

    if (show_manual) {
      for (const auto& vp : vposes_) {
        glLineWidth(std::get<2>(vp));
        pangolin::glDrawAxis(std::get<0>(vp), std::get<1>(vp));
        glLineWidth(1.0);
      }
    }

    s_cam.Apply();
    glColor3f(1.0, 1.0, 1.0);
    if (show_z0) pangolin::glDraw_z0(1.0, 2);
    glLineWidth(7.5);
    if (show_z0) pangolin::glDrawAxis(I_4x4, 0.11);
    glLineWidth(1.0);

    // ----------------
    // -- Stereo images.
    images_cam.Activate();
    glColor3f(1.0, 1.0, 1.0);

    imageTexture.Upload(imgL_.data, GL_BGR, GL_UNSIGNED_BYTE);
    imageTexture.RenderToViewport();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
}

/* ************************************************************************** */
void Visualizer::DrawObserver() const { DrawTrajectory(est_, 0.13); }

/* ************************************************************************** */
void Visualizer::DrawTarget() const {
  DrawTrajectory(tgt_);
  if (tgt_.size()) {
    std::vector<Eigen::Vector3d> CGline;
    Eigen::Vector3d tgtPos = tgt_.back().block<3, 1>(0, 3);
    CGline.push_back(tgtPos);
    CGline.push_back(CGpos_);
    glColor3f(0.0, 0.0, 0.0);
    glLineWidth(4.0);
    pangolin::glDrawLineStrip(CGline);
    glLineWidth(1.0);
    glColor3f(1.0, 1.0, 1.0);
  }
}

/* ************************************************************************** */
void Visualizer::DrawWorld(
    const Trajectory3& trajectory,
    const std::vector<Eigen::Vector3d>& landmarks) const {
  // Draw all poses.
  DrawTrajectory(trajectory);

  // Draw all landmarks.
  pangolin::glDrawPoints(landmarks);
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
void Visualizer::UpdateEstimate(const gtsam::Values& values,
                                const std::vector<double>& times) {
  vals_ = values;
  times_ = times;
}

}  // namespace mrg
