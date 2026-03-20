//
// Created by fu on 18/03/26.
//

#include "rerun_camera_camera_viz.h"

RerunCameraCameraViz::RerunCameraCameraViz(rerun::RecordingStream& rec,
                                           std::chrono::milliseconds throttle_ms)
    : rec_(rec), throttle_ms_(throttle_ms) {}

bool RerunCameraCameraViz::shouldLog(const std::string& view_name) {
    const auto now = std::chrono::steady_clock::now();
    auto& last = last_log_time_[view_name];
    if (now - last < throttle_ms_) return false;
    last = now;
    return true;
}

void RerunCameraCameraViz::vizDetections(const std::string& view_name,
                                         const std::vector<std::array<float, 2>>& corners_2d) {
    if (!shouldLog(view_name)) return;
    rec_.log(base_name_ + view_name + "/detections", rerun::Points2D(corners_2d));
}

void RerunCameraCameraViz::vizCovariances(const std::string& view_name,
                                           const Eigen::VectorXf& covariances) {
    if (!shouldLog(view_name)) return;
    (void)covariances;
}

void RerunCameraCameraViz::vizVoxelMap(const std::string &view_name, const std::vector<std::array<int32_t, 2>> &coords,
                                       const std::vector<uint32_t> &counts, float voxel_size, int width, int height) {
    if (coords.empty() || !shouldLog(view_name)) return;

    const auto vs = static_cast<int32_t>(voxel_size);

    // Image dimensions in original pixel space
    const int32_t img_width  = width;
    const int32_t img_height = height;

    std::vector<float> grid(static_cast<size_t>(img_width * img_height), 0.0f);
    for (size_t i = 0; i < coords.size(); ++i) {
        const int32_t col_origin = coords[i][0] * vs;
        const int32_t row_origin = coords[i][1] * vs;
        const float count = static_cast<float>(counts[i]);
        for (int32_t dy = 0; dy < vs; ++dy) {
            for (int32_t dx = 0; dx < vs; ++dx) {
                grid[static_cast<size_t>((row_origin + dy) * img_width + (col_origin + dx))] = count;
            }
        }
    }

    rec_.log(base_name_ + view_name,
             rerun::DepthImage(grid.data(), {static_cast<uint32_t>(img_width), static_cast<uint32_t>(img_height)}));
}

void RerunCameraCameraViz::vizFrameTransforms(const std::map<std::string, Eigen::Affine3d>& transforms) {
    rec_.log_static(base_name_, rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN); // Set an up-axis
    for (const auto& [name, T]: transforms) {
        const Eigen::Quaterniond q(T.rotation());
        const Eigen::Vector3d t = T.translation();

        rec_.log(
            base_name_ + name,
            rerun::Transform3D::from_translation_rotation(
                {static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z())},
                rerun::Quaternion::from_xyzw(
                    static_cast<float>(q.x()), static_cast<float>(q.y()),
                    static_cast<float>(q.z()), static_cast<float>(q.w()))
            ), rerun::TransformAxes3D(.05f)
        );
    }
}

void RerunCameraCameraViz::vizResidualMap(const std::string& view_name,
                                           const std::vector<std::array<float, 2>>& residuals_2d) {
    if (residuals_2d.empty() || !shouldLog(view_name)) return;

    const std::string path = base_name_ + view_name;
    rec_.log(path, rerun::Points2D(residuals_2d).with_colors(rerun::Color(34, 138, 167)));

    // Reference boxes showing 1 px and 3 px error bounds.
    rec_.log_static(path + "/1px",
                    rerun::Boxes2D::from_mins_and_sizes({{-1.f, -1.f}}, {{2.f, 2.f}})
                        .with_colors(rerun::Color(0, 255, 0)));
    rec_.log_static(path + "/3px",
                    rerun::Boxes2D::from_mins_and_sizes({{-3.f, -3.f}}, {{6.f, 6.f}})
                        .with_colors(rerun::Color(255, 0, 0)));
}

void RerunCameraCameraViz::vizDataAccumulationProgress(float percentage) {
    const auto view_name = std::string("data_accumulation_progress");
    percentage = std::min(std::max(percentage, 0.0f), 1.0f);
    const auto color = percentage >= 1.0f
        ? rerun::components::Color(255, 0, 0)    // red: done
        : rerun::components::Color(0, 255, 0); // gray: in progress
    // Log [percentage, 1.0f]: second value anchors y-axis to 1.0 so range is always [0, 1].
    rec_.log(base_name_ + view_name,
             rerun::BarChart::f32(std::vector<float>{percentage, 1.0f}).with_color(color));
}

void RerunCameraCameraViz::vizCalibrationSigmasPreamble() {
    rec_.log(base_name_ + std::string("calibration_sigmas"),
             rerun::TextDocument(calibration_sigmas_preamble_).with_media_type(rerun::MediaType::markdown()));
}

void RerunCameraCameraViz::vizAllCameraCalibrationSigmas(
        const std::map<std::string, std::pair<Eigen::VectorXd, Eigen::VectorXd>>& sigmas,
        const std::string& origin_camera,
        double intrinsics_threshold,
        double translation_threshold,
        double rotation_threshold) {
    std::string intrinsics_md =
        "## Intrinsics sigmas\n"
        "| | camera | fx | fy | cx | cy |\n"
        "|--|--------|----|----|----|----|  \n";
    std::string translation_md =
        "## Translation sigmas\n"
        "| | camera | tx | ty | tz |\n"
        "|--|--------|----|----|----|\n";
    std::string rotation_md =
        "## Rotation sigmas\n"
        "| | camera | rx | ry | rz |\n"
        "|--|--------|----|----|----|\n";

    for (const auto& [name, rt_fx] : sigmas) {
        const auto& rt = rt_fx.first;
        const auto& fx = rt_fx.second;
        char row[512];

        // --- intrinsics ---
        const double f0 = fx.size() > 0 ? fx(0) : 0.0;
        const double f1 = fx.size() > 1 ? fx(1) : 0.0;
        const double f2 = fx.size() > 2 ? fx(2) : 0.0;
        const double f3 = fx.size() > 3 ? fx(3) : 0.0;
        const bool intrinsics_ok = f0 < intrinsics_threshold && f1 < intrinsics_threshold &&
                                   f2 < intrinsics_threshold && f3 < intrinsics_threshold;
        const std::string intr_name = intrinsics_ok ?("**" + name + "**")  : name;
        std::snprintf(row, sizeof(row),
            "| %s | %s | %.2f | %.2f | %.2f | %.2f |\n",
            intrinsics_ok ? "✓" : " ", intr_name.c_str(), f0, f1, f2, f3);
        intrinsics_md += row;

        // --- translation ---
        const double t0 = rt.size() > 3 ? rt(3) : 0.0;
        const double t1 = rt.size() > 4 ? rt(4) : 0.0;
        const double t2 = rt.size() > 5 ? rt(5) : 0.0;
        const bool is_origin       = (name == origin_camera);
        const bool t_all_zero      = (t0 == 0.0 && t1 == 0.0 && t2 == 0.0);
        const bool t_not_seen      = t_all_zero && !is_origin;
        const bool t_ok            = !t_not_seen && t0 < translation_threshold &&
                                     t1 < translation_threshold && t2 < translation_threshold;
        const std::string t_name   = t_not_seen ? ("~~" + name + "~~")
                                   : t_ok       ? ("**" + name + "**")
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

        // --- rotation ---
        const double r0 = rt.size() > 0 ? rt(0) : 0.0;
        const double r1 = rt.size() > 1 ? rt(1) : 0.0;
        const double r2 = rt.size() > 2 ? rt(2) : 0.0;
        const bool r_all_zero      = (r0 == 0.0 && r1 == 0.0 && r2 == 0.0);
        const bool r_not_seen      = r_all_zero && !is_origin;
        const bool r_ok            = !r_not_seen && r0 < rotation_threshold &&
                                     r1 < rotation_threshold && r2 < rotation_threshold;
        const std::string r_name   = r_not_seen ? ("~~" + name + "~~")
                                   : r_ok       ? ("**" + name + "**")
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

    const std::string md = std::string(calibration_sigmas_preamble_) + intrinsics_md + "\n" + translation_md + "\n" + rotation_md;
    rec_.log(base_name_ + std::string("calibration_sigmas"),
             rerun::TextDocument(md).with_media_type(rerun::MediaType::markdown()));
}

void RerunCameraCameraViz::vizAdjacencyGraph(const std::string& view_name,
                                              const std::vector<std::string>& nodes,
                                              const std::vector<std::pair<std::string, std::string>>& edges,
                                              const std::vector<int>& edge_counts) {
    if (!shouldLog(view_name)) return;

    std::vector<std::string> node_labels;
    node_labels.reserve(nodes.size());
    for (const auto& n : nodes) {
        node_labels.push_back(n);
    }

    std::vector<rerun::components::GraphEdge> graph_edges;
    graph_edges.reserve(edges.size());
    std::vector<std::string> edge_labels;
    edge_labels.reserve(edges.size());
    for (size_t i = 0; i < edges.size(); ++i) {
        graph_edges.push_back({edges[i].first, edges[i].second});
        edge_labels.push_back(std::to_string(edge_counts[i]));
    }

    rec_.log(base_name_ + view_name,
             rerun::GraphNodes(nodes).with_labels(node_labels),
             rerun::GraphEdges(graph_edges)
                 .with_graph_type(rerun::components::GraphType::Undirected));
}