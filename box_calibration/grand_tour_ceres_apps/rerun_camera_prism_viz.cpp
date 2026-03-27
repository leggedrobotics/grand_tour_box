#include "rerun_camera_prism_viz.h"
#include <algorithm>

const std::vector<rerun::Color> RerunCameraPrismViz::kCameraColors = {
        {138, 63,  252},  // purple
        {51,  177, 255},  // cyan
        {250, 77,  86},  // red
        {111, 220, 140},  // green
        {209, 39,  113},  // magenta
        {160, 50,  200},  // purple
        {200, 100, 150},  // pink
};

RerunCameraPrismViz::RerunCameraPrismViz(rerun::RecordingStream &rec) : rec_(rec) {}

static std::string sanitizeName(const std::string &name) {
    std::string out = name;
    std::replace(out.begin(), out.end(), '/', '_');
    return out;
}

rerun::Color RerunCameraPrismViz::colorForCamera(const std::string &camera_name) {
    if (camera_color_index_.find(camera_name) == camera_color_index_.end()) {
        camera_color_index_[camera_name] = camera_color_index_.size();
    }
    return kCameraColors[camera_color_index_.at(camera_name) % kCameraColors.size()];
}

void RerunCameraPrismViz::logStripAndTimeSeries(
        const std::string &strip_entity,
        const std::string &series_parent,
        const std::string &series_name,
        const rerun::Color &color,
        const std::vector<double> &timestamps_s,
        const std::vector<std::array<float, 3>> &positions) {

    // Per-axis scalar time series. Paths are series_parent/x/series_name etc. so that
    // all sources sharing the same series_parent are co-plotted on the same chart.
    const std::string sx = series_parent + "/x/" + series_name;
    const std::string sy = series_parent + "/y/" + series_name;
    const std::string sz = series_parent + "/z/" + series_name;
    rec_.log_static(sx, rerun::SeriesLines()
            .with_colors(color)
            .with_widths(2));
    rec_.log_static(sy, rerun::SeriesLines()
            .with_colors(color)
            .with_widths(2));
    rec_.log_static(sz, rerun::SeriesLines()
            .with_colors(color)
            .with_widths(2));
    for (size_t i = 0; i < timestamps_s.size(); ++i) {
        rec_.set_time_timestamp_secs_since_epoch("prism_data_timeline", timestamps_s[i]);
        rec_.log(sx, rerun::Scalars(positions[i][0]));
        rec_.log(sy, rerun::Scalars(positions[i][1]));
        rec_.log(sz, rerun::Scalars(positions[i][2]));
    }
    // 3D line strip — logged timeless so it always appears regardless of timeline position.
    std::vector<rerun::components::Position3D> pts;
    pts.reserve(positions.size());
    for (const auto &p: positions) {
        pts.push_back({p[0], p[1], p[2]});
    }
    rec_.log_static(strip_entity,
                    rerun::Points3D(pts)
                            .with_colors(color)
                            .with_radii(0.005f)
                            .with_labels(rerun::Text(series_name))
                            .with_show_labels(true));

}

void RerunCameraPrismViz::vizStaticFrames(const Eigen::Affine3d &T_leica_board) {
    rec_.log_static(base_name_, rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

    // Total station / leica frame is the world origin.
    rec_.log_static(
            std::string(base_name_) + "leica",
            rerun::Transform3D::from_translation_rotation(
                    {0.f, 0.f, 0.f},
                    rerun::Quaternion::from_xyzw(0.f, 0.f, 0.f, 1.f)),
            rerun::TransformAxes3D(1.0f).with_show_frame(true));

    // Calibration board — fixed in the world at T_leica_board.
    const Eigen::Quaterniond q(T_leica_board.rotation());
    const Eigen::Vector3d t = T_leica_board.translation();
    rec_.log_static(
            std::string(base_name_) + "board",
            rerun::Transform3D::from_translation_rotation(
                    {float(t.x()), float(t.y()), float(t.z())},
                    rerun::Quaternion::from_xyzw(
                            float(q.x()), float(q.y()), float(q.z()), float(q.w()))),
            rerun::TransformAxes3D(1.0f).with_show_frame(true));
}

void RerunCameraPrismViz::vizPrismObservedTrajectory(
        const std::vector<double> &timestamps_s,
        const std::vector<std::array<float, 3>> &positions) {
    if (positions.empty()) return;
    logStripAndTimeSeries(
            std::string(base_name_) + "prism/observed/strip",
            std::string(base_name_) + "prism",
            "observed",
            rerun::Color(241, 194, 27),  // yellow
            timestamps_s, positions);
}

void RerunCameraPrismViz::vizPrismInterpolatedAtCameraTime(
        const std::vector<double> &timestamps_s,
        const std::vector<std::array<float, 3>> &positions) {
    if (positions.empty()) return;
    logStripAndTimeSeries(
            std::string(base_name_) + "prism/interpolated/strip",
            std::string(base_name_) + "prism",
            "interpolated",
            rerun::Color(255, 255, 100),  // yellow — same prism, resampled to camera times
            timestamps_s, positions);
}

void RerunCameraPrismViz::vizPrismEstimatedByCameraDetections(
        const std::string &camera_name,
        const std::vector<double> &timestamps_s,
        const std::vector<std::array<float, 3>> &positions) {
    if (positions.empty()) return;
    const std::string safe = sanitizeName(camera_name);
    logStripAndTimeSeries(
            std::string(base_name_) + "prism/estimated/" + safe + "/strip",
            std::string(base_name_) + "prism",
            safe,
            colorForCamera(camera_name),
            timestamps_s, positions);
}

void RerunCameraPrismViz::vizCameraTrajectory(
        const std::string &camera_name,
        const std::vector<double> &timestamps_s,
        const std::vector<Eigen::Affine3d> &T_leica_cam_over_time) {
    if (T_leica_cam_over_time.empty()) return;

    const std::string base = std::string(base_name_) + "cameras/" + sanitizeName(camera_name);
    const auto color = colorForCamera(camera_name);

    // 3D line strip of camera centre — logged timeless.
    std::vector<rerun::components::Position3D> pts;
    pts.reserve(T_leica_cam_over_time.size());
    for (const auto &T: T_leica_cam_over_time) {
        const Eigen::Vector3d t = T.translation();
        pts.push_back({float(t.x()), float(t.y()), float(t.z())});
    }
    rec_.log(base + "/strip", rerun::LineStrips3D(pts).with_colors(color));

    // Transform (orientation + position) and per-axis scalars at each timestamp.
    for (size_t i = 0; i < timestamps_s.size(); ++i) {
        rec_.set_time_timestamp_secs_since_epoch("prism_data_timeline", timestamps_s[i]);

        const Eigen::Vector3d t = T_leica_cam_over_time[i].translation();
        const Eigen::Quaterniond q(T_leica_cam_over_time[i].rotation());

        rec_.log(base + "/x", rerun::Scalars(t.x()));
        rec_.log(base + "/y", rerun::Scalars(t.y()));
        rec_.log(base + "/z", rerun::Scalars(t.z()));

        rec_.log(base,
                 rerun::Transform3D::from_translation_rotation(
                         {float(t.x()), float(t.y()), float(t.z())},
                         rerun::Quaternion::from_xyzw(
                                 float(q.x()), float(q.y()), float(q.z()), float(q.w()))),
                 rerun::TransformAxes3D(0.05f));
    }
}

void RerunCameraPrismViz::vizCalibrationSigmas(
        const std::map<std::string, Eigen::VectorXd> &rt_sigmas,
        const std::string &origin_camera,
        double translation_threshold,
        double rotation_threshold) {

    std::string translation_md =
            "## Translation sigmas (m)\n"
            "| | camera | tx | ty | tz |\n"
            "|--|--------|----|----|----|\n";
    std::string rotation_md =
            "## Rotation sigmas (rad)\n"
            "| | camera | rx | ry | rz |\n"
            "|--|--------|----|----|----|\n";

    for (const auto &[name, rt]: rt_sigmas) {
        char row[512];
        const bool is_origin = (name == origin_camera);

        // Translation: indices 3, 4, 5
        const double t0 = rt.size() > 3 ? rt(3) : 0.0;
        const double t1 = rt.size() > 4 ? rt(4) : 0.0;
        const double t2 = rt.size() > 5 ? rt(5) : 0.0;
        const bool t_all_zero = (t0 == 0.0 && t1 == 0.0 && t2 == 0.0);
        const bool t_not_seen = t_all_zero && !is_origin;
        const bool t_ok = !t_not_seen && t0 < translation_threshold &&
                          t1 < translation_threshold && t2 < translation_threshold;
        const std::string t_name = t_not_seen ? ("~~" + name + "~~")
                                              : t_ok ? ("**" + name + "**")
                                                     : name;
        if (t_not_seen)
            std::snprintf(row, sizeof(row),
                          "| | %s | ~~%.4f~~ | ~~%.4f~~ | ~~%.4f~~ |\n",
                          t_name.c_str(), t0, t1, t2);
        else
            std::snprintf(row, sizeof(row),
                          "| %s | %s | %.4f | %.4f | %.4f |\n",
                          t_ok ? "✓" : " ", t_name.c_str(), t0, t1, t2);
        translation_md += row;

        // Rotation: indices 0, 1, 2
        const double r0 = rt.size() > 0 ? rt(0) : 0.0;
        const double r1 = rt.size() > 1 ? rt(1) : 0.0;
        const double r2 = rt.size() > 2 ? rt(2) : 0.0;
        const bool r_all_zero = (r0 == 0.0 && r1 == 0.0 && r2 == 0.0);
        const bool r_not_seen = r_all_zero && !is_origin;
        const bool r_ok = !r_not_seen && r0 < rotation_threshold &&
                          r1 < rotation_threshold && r2 < rotation_threshold;
        const std::string r_name = r_not_seen ? ("~~" + name + "~~")
                                              : r_ok ? ("**" + name + "**")
                                                     : name;
        if (r_not_seen)
            std::snprintf(row, sizeof(row),
                          "| | %s | ~~%.4f~~ | ~~%.4f~~ | ~~%.4f~~ |\n",
                          r_name.c_str(), r0, r1, r2);
        else
            std::snprintf(row, sizeof(row),
                          "| %s | %s | %.4f | %.4f | %.4f |\n",
                          r_ok ? "✓" : " ", r_name.c_str(), r0, r1, r2);
        rotation_md += row;
    }

    const std::string md = std::string(calibration_sigmas_preamble_) +
                           translation_md + "\n" + rotation_md;
    rec_.log(std::string(base_name_) + "calibration_sigmas",
             rerun::TextDocument(md).with_media_type(rerun::MediaType::markdown()));
}

void RerunCameraPrismViz::vizCalibrationResults(
        const std::string &origin_camera,
        const Eigen::Vector3d &t_cam0_prism,
        double time_offset,
        const Eigen::Matrix4d &T_totalstation_board,
        const Eigen::Vector3d &prism_position_sigma,
        const Eigen::VectorXd &board_pose_sigma,
        double time_offset_sigma) {

    constexpr double rad2deg = 180.0 / M_PI;
    constexpr double m_in_mm = 1000.0;

    char buf[512];

    // --- t_cam0_prism ---
    std::string md = "# Calibration Results\n\n"
                     "Prism position in '" + origin_camera + "' frame (m)\n"
                                                             "| tx | ty | tz |\n"
                                                             "|----|----|----|  \n";
    std::snprintf(buf, sizeof(buf), "| **%.5g** | **%.5g** | **%.5g** |\n\n",
                  t_cam0_prism.x(), t_cam0_prism.y(), t_cam0_prism.z());
    md += buf;

    // --- time offset ---
    md += "Time offset \n"
          "| value (s) |\n"
          "|-------|\n";
    std::snprintf(buf, sizeof(buf), "| %.6g |\n\n", time_offset);
    md += buf;

    // --- T_totalstation_board ---
    md += "T_totalstation_board - Static board pose in total station frame`\n"
          "|   | c0 | c1 | c2 | c3 |\n"
          "|---|----|----|----|----|  \n";
    for (int r = 0; r < 4; ++r) {
        std::snprintf(buf, sizeof(buf),
                      "| r%d | %.4g | %.4g | %.4g | %.4g |\n",
                      r,
                      T_totalstation_board(r, 0), T_totalstation_board(r, 1),
                      T_totalstation_board(r, 2), T_totalstation_board(r, 3));
        md += buf;
    }
    md += "\n";

    // --- Sigmas ---
    md += "Prism position σ\n"
          "| σx (mm) | σy (mm) | σz (mm) |\n"
          "|----|----|----|  \n";
    std::snprintf(buf, sizeof(buf), "| %.2g | %.2g | %.2g |\n\n",
                  prism_position_sigma.x() * m_in_mm, prism_position_sigma.y() * m_in_mm,
                  prism_position_sigma.z() * m_in_mm);
    md += buf;


    md += "Board pose σ \n"
          "| rx (deg) | ry (deg) | rz (deg) | tx (mm) | ty (mm) | tz (mm) |\n"
          "|-----|-----|-----|-----|-----|-----|  \n";
    const double sr0 = board_pose_sigma.size() > 0 ? board_pose_sigma(0) : 0.0;
    const double sr1 = board_pose_sigma.size() > 1 ? board_pose_sigma(1) : 0.0;
    const double sr2 = board_pose_sigma.size() > 2 ? board_pose_sigma(2) : 0.0;
    const double st0 = board_pose_sigma.size() > 3 ? board_pose_sigma(3) : 0.0;
    const double st1 = board_pose_sigma.size() > 4 ? board_pose_sigma(4) : 0.0;
    const double st2 = board_pose_sigma.size() > 5 ? board_pose_sigma(5) : 0.0;
    std::snprintf(buf, sizeof(buf),
                  "| %.3g | %.3g | %.3g | %.2g | %.2g | %.2g |\n\n",
                  sr0 * rad2deg, sr1 * rad2deg, sr2 * rad2deg, st0 * m_in_mm, st1 * m_in_mm, st2 * m_in_mm);
    md += buf;

    md += "Time offset σ (s)\n"
          "| value |\n"
          "|-------|\n";
    std::snprintf(buf, sizeof(buf), "| %.4g |\n", time_offset_sigma);
    md += buf;
    md += "\n\nThe residual plots show the **+/- 1mm** bounds (green), and the **+/- 3mm** bounds (red).\n\n";

    rec_.log_static(std::string(base_name_) + "calibration_results",
                    rerun::TextDocument(md).with_media_type(rerun::MediaType::markdown()));
}

void RerunCameraPrismViz::vizResiduals3D(
        const std::map<std::string, std::vector<Eigen::Vector3d>> &residuals_per_camera) {

    constexpr float m_to_mm = 1000.f;

    for (const auto &[camera_name, residuals]: residuals_per_camera) {
        if (residuals.empty()) continue;

        const std::string safe = sanitizeName(camera_name);
        const auto default_color = colorForCamera(camera_name);
        const auto transparent_color = rerun::Color(default_color.r(), default_color.g(), default_color.b(),
                                                    200);

        std::vector<rerun::components::Position2D> xy_pts, yz_pts, xz_pts;
        xy_pts.reserve(residuals.size());
        yz_pts.reserve(residuals.size());
        xz_pts.reserve(residuals.size());

        for (const auto &r: residuals) {
            const float rx = static_cast<float>(r.x()) * m_to_mm;
            const float ry = static_cast<float>(r.y()) * m_to_mm;
            const float rz = static_cast<float>(r.z()) * m_to_mm;
            xy_pts.push_back({rx, ry});
            yz_pts.push_back({ry, rz});
            xz_pts.push_back({rx, rz});
        }

        rec_.log_static(std::string(base_name_) + "XY_residuals/" + safe,
                        rerun::Points2D(xy_pts)
                                .with_colors(transparent_color).with_radii(0.1f)
                                .with_labels(rerun::Text(safe)).with_draw_order(0));
        rec_.log_static(std::string(base_name_) + "YZ_residuals/" + safe,
                        rerun::Points2D(yz_pts)
                                .with_colors(transparent_color).with_radii(0.1f)
                                .with_labels(rerun::Text(safe)).with_draw_order(0));
        rec_.log_static(std::string(base_name_) + "XZ_residuals/" + safe,
                        rerun::Points2D(xz_pts)
                                .with_colors(transparent_color).with_radii(0.1f)
                                .with_labels(rerun::Text(safe)).with_draw_order(0));

        // Reference boxes showing 1 px and 3 mm error bounds.
        rec_.log_static(std::string(base_name_) + "XY_residuals/1mm",
                        rerun::Boxes2D::from_mins_and_sizes({{-1.f, -1.f}}, {{2.f, 2.f}})
                                .with_colors(rerun::Color(0, 255, 0)));
        rec_.log_static(std::string(base_name_) + "XY_residuals/3mm",
                        rerun::Boxes2D::from_mins_and_sizes({{-3.f, -3.f}}, {{6.f, 6.f}})
                                .with_colors(rerun::Color(255, 0, 0)));


        // Reference boxes showing 1 px and 3 mm error bounds.
        rec_.log_static(std::string(base_name_) + "YZ_residuals/1mm",
                        rerun::Boxes2D::from_mins_and_sizes({{-1.f, -1.f}}, {{2.f, 2.f}})
                                .with_colors(rerun::Color(0, 255, 0)));
        rec_.log_static(std::string(base_name_) + "YZ_residuals/3mm",
                        rerun::Boxes2D::from_mins_and_sizes({{-3.f, -3.f}}, {{6.f, 6.f}})
                                .with_colors(rerun::Color(255, 0, 0)));


        // Reference boxes showing 1 px and 3 mm error bounds.
        rec_.log_static(std::string(base_name_) + "XZ_residuals/1mm",
                        rerun::Boxes2D::from_mins_and_sizes({{-1.f, -1.f}}, {{2.f, 2.f}})
                                .with_colors(rerun::Color(0, 255, 0)));
        rec_.log_static(std::string(base_name_) + "XZ_residuals/3mm",
                        rerun::Boxes2D::from_mins_and_sizes({{-3.f, -3.f}}, {{6.f, 6.f}})
                                .with_colors(rerun::Color(255, 0, 0)));
    }
}