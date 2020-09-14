/**
 * @file Visualizer.h
 * @brief Simple multi-threaded 3D graph visualization based on Pangolin.
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */

#include "tonioviz/Visualizer.h"

#include <Eigen/StdVector>

namespace mrg {

Visualizer::Visualizer(const float w, const float h, const float f)
    : w_(w), h_(h), f_(f) {
  // Make sure the matched is never a null pointer.
  matched_ = cv::Mat(480, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
  CGpos_ = Eigen::Vector3d::Zero();

  // Load the ground truth gyroscope data.
  std::string dataset = "/Users/tonio/data/vertigomod/TS53T7R3mod/";
  std::string imu_file = dataset + "GSdata/tgt_imu_data_gyroonly.txt";
  std::vector<MsgImuData> imu = LoadImuData(imu_file);

  gyros_.reserve(imu.size());
  gyrotimes_.reserve(imu.size());
  double gyro_start_time = 335, gyro_end_time = 450;
  for (const sph::MsgImuData& m : imu) {
    if (m.sph_test_time >= gyro_start_time &&
        m.sph_test_time <= gyro_end_time) {
      gyros_.push_back(m.gyro);
      gyrotimes_.push_back(m.vertigo_test_time);
    } else if (m.sph_test_time > gyro_end_time) {
      break;
    }
  }

  // Initialize to origin the principal axes orientation.
  R_BG_ = Eigen::Matrix3d::Identity();
}

Visualizer::~Visualizer() {}

void Visualizer::RenderWorld() {
  std::cout << "Starting the visualization thread." << std::endl;
  pangolin::CreateWindowAndBind("spheres-vertigo viewer", w_, h_);
  glEnable(GL_DEPTH_TEST);

  // TODO(tonioteran) Figure out what the rest of the hardcoded params are, and
  // abstract for manual configuration using an external file?

  // Define Projection and initial ModelView matrix.
  pangolin::OpenGlMatrix proj =
      pangolin::ProjectionMatrix(w_, h_, f_, f_, w_ / 2.0, h_ / 2.0, 0.2, 1000);
  pangolin::OpenGlRenderState s_cam(
      proj,
      pangolin::ModelViewLookAt(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, pangolin::AxisZ));
  pangolin::OpenGlRenderState s_cam_polhode(
      proj,
      pangolin::ModelViewLookAt(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, pangolin::AxisZ));
  pangolin::OpenGlRenderState s_cam_isync(
      proj,
      pangolin::ModelViewLookAt(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, pangolin::AxisZ));

  float midlinef = 0.4;
  pangolin::Attach midline(midlinef), top(1.0);
  pangolin::Attach imgline = std::min(midlinef * (1280.0 / 480.0), 0.6);

  // Create the individual cameras for each view.
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.4, 1.0, 0.0, 0.66)
                              .SetHandler(new pangolin::Handler3D(s_cam));
  pangolin::View& polhode_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.4, 0.625, 0.66, 1.0)
          .SetHandler(new pangolin::Handler3D(s_cam_polhode));
  pangolin::View& isync_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.625, 1.0, 0.66, 1.0)
          .SetHandler(new pangolin::Handler3D(s_cam_isync));
  pangolin::View& images_cam =
      pangolin::CreateDisplay().SetBounds(0.0, 0.4, 0.0, imgline);

  // Create the 2D plots for the angular velocity estimates.
  float min_x = 0.0, max_x = 180;
  pangolin::Plotter plotter_x(&omegaxLog_, min_x, max_x);
  plotter_x.SetBounds(0.0, midline, imgline, 1.0);
  plotter_x.Track("$i");

  pangolin::DisplayBase().AddDisplay(plotter_x);

  // Real-time toggles using key presses.
  bool show_z0 = true;
  pangolin::RegisterKeyPressCallback('z', [&]() { show_z0 = !show_z0; });

  bool show_gtsam = true;
  pangolin::RegisterKeyPressCallback('g', [&]() { show_gtsam = !show_gtsam; });

  bool show_manual = true;
  pangolin::RegisterKeyPressCallback('m',
                                     [&]() { show_manual = !show_manual; });

  bool show_frames = true;
  pangolin::RegisterKeyPressCallback('f',
                                     [&]() { show_frames = !show_frames; });

  // Manage the size of the points.
  glPointSize(3.5);  // Default is 1.
  // Useful identity.
  Eigen::Matrix4d I_4x4 = Eigen::Matrix4d::Identity();

  // Deal with the VERTIGO images.
  const int width = 1280;
  const int height = 480;
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

    if (show_frames) DrawFrames(vframes_);

    s_cam.Apply();
    glColor3f(1.0, 1.0, 1.0);
    if (show_z0) pangolin::glDraw_z0(1.0, 2);
    glLineWidth(7.5);
    if (show_z0) pangolin::glDrawAxis(I_4x4, 0.11);
    glLineWidth(1.0);

    // ----------------
    // -- Polhode plot.
    polhode_cam.Activate(s_cam_polhode);

    // Draw the raw measured points.
    glPointSize(5.0);  // Default is 1.
    glColor3f(0.7, 0.5, 0.5);
    if (omegas_.size()) pangolin::glDrawPoints(omegas_);
    // Draw line connecting all measurements.
    glColor4f(0.8, 0.6, 0.6, 0.5);
    glLineWidth(1.0);
    pangolin::glDrawLineStrip(omegas_);
    glColor3f(1.0, 1.0, 1.0);

    // Draw the raw measured points.
    glPointSize(5.0);  // Default is 1.
    glColor3f(0.7, 0.5, 0.9);
    if (rotomegas_.size()) pangolin::glDrawPoints(rotomegas_);
    // Draw line connecting all measurements.
    glColor4f(0.65, 0.45, 0.9, 0.5);
    glLineWidth(1.0);
    pangolin::glDrawLineStrip(rotomegas_);
    glColor3f(1.0, 1.0, 1.0);

    DrawGyroPolhode();

    glColor3f(1.0, 1.0, 1.0);
    if (show_z0) pangolin::glDraw_z0(1.0, 1);
    glLineWidth(5.0);
    if (show_z0) pangolin::glDrawAxis(I_4x4, 0.25);

    // ----------------
    // -- iSync camera.
    isync_cam.Activate(s_cam_isync);
    glLineWidth(1.0);
    glColor3f(1.0, 1.0, 1.0);

    if (show_gtsam) {
      DrawTrajectory(estWrtG_);
    }

    if (show_z0) pangolin::glDraw_z0(1.0, 1);
    glLineWidth(5.0);
    if (show_z0) pangolin::glDrawAxis(I_4x4, 0.25);

    // ----------------
    // -- Stereo images.
    images_cam.Activate();
    glColor3f(1.0, 1.0, 1.0);

    imageTexture.Upload(matched_.data, GL_BGR, GL_UNSIGNED_BYTE);
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

  // Extract all state estimates.
  size_t counter = 0;
  est_.clear();
  tgt_.clear();
  estWrtG_.clear();
  while (vals_.exists(gtsam::Symbol('T', counter))) {
    // Inpector's states.
    gtsam::Pose3 T_WBi = vals_.at<gtsam::Pose3>(gtsam::Symbol('T', counter));
    est_.push_back(T_WBi.matrix());

    // Target trajectory.
    gtsam::Pose3 T_GiBi = vals_.at<gtsam::Pose3>(gtsam::Symbol('G', counter));
    estWrtG_.push_back(T_GiBi.matrix());
    gtsam::Pose3 T_WGi = T_WBi * T_GiBi.inverse();
    tgt_.push_back(T_WGi.matrix());

    counter++;
  }

  // Extract the center of mass offset.
  if (vals_.exists(gtsam::Symbol('t', 0))) {
    Eigen::Vector3d t_BwrtG_G =
        vals_.at<gtsam::Point3>(gtsam::Symbol('t', 0)).vector();

    // Extract latest target pose.
    Eigen::Matrix4d T_WG = tgt_.back();
    Eigen::Vector3d t_GwrtW_W = T_WG.block<3, 1>(0, 3);
    Eigen::Matrix3d R_WG = T_WG.block<3, 3>(0, 0);

    // Location of the center of mass.
    CGpos_ = t_GwrtW_W + R_WG * t_BwrtG_G;
  }

  // Extract the optimal principal axes orientation.
  if (vals_.exists(gtsam::Symbol('R', 0))) {
    R_BG_ = vals_.at<gtsam::Rot3>(gtsam::Symbol('R', 0)).matrix();
  }

  // Extract angular velocity history.
  omegaxLog_.Clear();
  omegas_.clear();
  rotomegas_.clear();

  // Export the estimated omegas to file.
  std::string omegaFile =
      "/Users/tonio/Documents/MATLAB/matlab/dynamics/out.Gomegas";
  std::ofstream outGomega(omegaFile);

  size_t num_poses = tgt_.size();
  for (int j = 1; j < num_poses; j++) {
    int i = j - 1;

    Eigen::Matrix3d Ri = tgt_[i].block<3, 3>(0, 0);
    Eigen::Matrix3d Rj = tgt_[j].block<3, 3>(0, 0);
    double deltaT = (times_[j] - times[i]) / 1000.0;  // [s]

    // To avoid taking into account non-adjacent time steps.
    if (deltaT < 0.8) {
      // TEMP TODO(tonio) Hardcoding the frame rate!!!
      Eigen::Vector3d omega = ComputeAngularVelocity(Ri, Rj, 0.5);
      omegaxLog_.Log(static_cast<float>(omega(0)), static_cast<float>(omega(1)),
                     static_cast<float>(omega(2)));
      omegas_.push_back(omega);
      rotomegas_.push_back(R_BG_.transpose() * omega);

      // Output to file.
      // Get average time to include in the output file.
      double avgTime = 0.5 * (times_[i] + times_[j]) / 1000.0;  // [s]
      outGomega << avgTime << " " << omega.transpose() << std::endl;
    }
  }

  outGomega.close();
}

/* *************************************************************************  */
void Visualizer::DrawGyroPolhode() const {
  glColor4f(0.4, 0.7, 0.9, 0.1);
  glLineWidth(3.0);
  pangolin::glDrawLineStrip(gyros_);
  glLineWidth(1.0);
  glColor3f(1.0, 1.0, 1.0);
}

/* *************************************************************************  */
void Visualizer::DrawFrames(const std::vector<VizFrame> vframes) const {
  std::vector<Eigen::Vector3d> pts;
  for (const auto& vf : vframes) {
    // Extract all points.
    pts.clear();
    pts.reserve(std::get<0>(vf).leftRightMatches3D.size());
    for (const auto& p : std::get<0>(vf).leftRightMatches3D) {
      pts.push_back(p);
    }

    // Visualize.
    glPointSize(std::get<4>(vf));
    glColor3f(std::get<1>(vf), std::get<2>(vf), std::get<3>(vf));
    pangolin::glDrawPoints(pts);
    glPointSize(1.0);
  }
}

}  // namespace sph
