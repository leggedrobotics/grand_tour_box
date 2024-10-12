//
// Created by fu on 05/09/2024.
//

#include <gtboxcalibration/utils.h>

int dummy_function_for_tests_addition(int a, int b) {
    return a + b;
}

CameraCamera2D3DTargetDetectionData FetchMulticamera2D3DDetectionData(std::string path) {
    nlohmann::json data;
    {
        std::ifstream f(path);
        data = nlohmann::json::parse(f);
    }
    CameraCamera2D3DTargetDetectionData result;
    std::vector<unsigned long long> stamps = data["stamps"];
    result.unique_timestamps = std::set<unsigned long long>(stamps.begin(), stamps.end());

    for (const auto &stamp: result.unique_timestamps) {
        for (const auto &data_at_stamp: data[std::to_string(stamp)]) {
            std::string cam_name = data_at_stamp["rostopic"];
            Eigen::Matrix2Xd corners2d(2, data_at_stamp["corners2d"].size());
            for (int i = 0; i < corners2d.cols(); i++) {
                const auto &point = data_at_stamp["corners2d"][i];
                corners2d(0, i) = point[0];
                corners2d(1, i) = point[1];
            }
            Eigen::Matrix3Xd model_points(3, data_at_stamp["model_points"].size());
            for (int i = 0; i < model_points.cols(); i++) {
                const auto &point = data_at_stamp["model_points"][i];
                model_points(0, i) = point[0];
                model_points(1, i) = point[1];
                model_points(2, i) = point[2];
            }

            Eigen::Affine3d T_cam_board = Eigen::Affine3d::Identity();
            const auto T_data = data_at_stamp["T_camera_board"];
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    T_cam_board(i, j) = T_data[i][j];
                }
            }

            std::vector<unsigned int> ids(corners2d.cols());

            result.observations[stamp][cam_name] = {
                    corners2d, model_points, ids, T_cam_board, cam_name
            };
        }
    }
    return result;
}

std::map<std::string, CameraParameterPack>
PopulateCameraParameterPacks(const std::string& intrinsics_yaml_path, const std::string& extrinsics_path) {

    const YAML::Node kalibr_data = YAML::LoadFile(intrinsics_yaml_path);
    const auto extrinsics = FetchExtrinsicsFromYamlPath(extrinsics_path);

    std::map<std::string, CameraParameterPack> result = FetchIntrinsicsAsCameraPackFromYaml(kalibr_data);
    for (const auto& [name, _ ] : result) {
        const auto transform = extrinsics.at(name);
        SE3Transform::assignToData(transform, result[name].T_bundle_sensor);
    }

    return result;
}

std::map<std::string, CameraParameterPack> FetchIntrinsicsAsCameraPackFromYaml(const YAML::Node &kalibr_data) {
    std::map<std::string, CameraParameterPack> result;
    for (YAML::const_iterator it = kalibr_data.begin(); it != kalibr_data.end(); ++it) {
        const auto &cam_data = it->second;
        const auto rostopic = cam_data["rostopic"].as<std::string>();
        const auto distortion_model = cam_data["distortion_model"].as<std::string>();
        const auto intrinsics = cam_data["intrinsics"].as<std::vector<double>>();
        const auto distortion_coeffs = cam_data["distortion_coeffs"].as<std::vector<double>>();
        const auto resolution = cam_data["resolution"].as<std::vector<int>>();

        memcpy(result[rostopic].fxfycxcy,
                    intrinsics.data(), PinholeProjection::NUM_PARAMETERS * sizeof(double));
        memcpy(result[rostopic].dist_coeffs,
                    distortion_coeffs.data(), Distortion::NUM_PARAMETERS * sizeof(double));
        result[rostopic].width = resolution[0];
        result[rostopic].height = resolution[1];

        if (cam_data["distortion_model"].as<std::string>() == "equidistant") {
            result[rostopic].distortion_type = Distortion::Fisheye;
        } else if (cam_data["distortion_model"].as<std::string>() == "radtan") {
            result[rostopic].distortion_type = Distortion::RadTan;
        } else {
            std::cerr << "Tried to populate wrong distortion model" << std::endl;
//            return {};
        }
    }
    return result;
}

std::map<std::string, Eigen::Affine3d> FetchExtrinsicsFromYamlPath(std::string yaml_path) {
    const YAML::Node extrinsics_data = YAML::LoadFile(yaml_path);
    return FetchExtrinsicsFromYaml(extrinsics_data);
}

std::map<std::string, Eigen::Affine3d> FetchExtrinsicsFromYaml(const YAML::Node &extrinsics_data) {
    std::map<std::string, Eigen::Affine3d> eigen_results;
    for (YAML::const_iterator it = extrinsics_data.begin(); it != extrinsics_data.end(); ++it) {
        const auto &cam_data = it->second;
        const auto &rostopic = cam_data["rostopic"].as<std::string>();
        YAML::Node bundle_cam_data = cam_data["T_bundle_camera"];
        Eigen::Matrix4d T_bundle_cam = Eigen::Matrix4d::Identity();
        for (std::size_t j = 0; j < bundle_cam_data.size(); ++j) {
            YAML::Node row = bundle_cam_data[j];
            for (std::size_t k = 0; k < row.size(); ++k) {
                T_bundle_cam(j, k) = row[k].as<double>();
            }
        }
        Eigen::Affine3d output_T_bundle_cam;
        output_T_bundle_cam.translation() = T_bundle_cam.block<3, 1>(0, 3);
        output_T_bundle_cam.linear() = T_bundle_cam.block<3, 3>(0, 0);
        eigen_results[rostopic] = output_T_bundle_cam;
    }
    return eigen_results;
}

bool SerialiseCameraParameters(const std::string output_path,
                               const std::map<std::string, CameraParameterPack> &camera_parameter_packs) {
    YAML::Node node;

    int index = 0;
    for (const auto&[cam_name, pack]: camera_parameter_packs) {
        YAML::Node cam_node;

        YAML::Node dist_coeffs = YAML::Node(YAML::NodeType::Sequence);
        for (auto val: pack.dist_coeffs) {
            dist_coeffs.push_back(val);
        }

        YAML::Node intrinsics = YAML::Node(YAML::NodeType::Sequence);
        for (auto val: pack.fxfycxcy) {
            intrinsics.push_back(val);
        }

        YAML::Node resolution = YAML::Node(YAML::NodeType::Sequence);
        resolution.push_back(pack.width);
        resolution.push_back(pack.height);

        YAML::Node T_bundle_cam_node = YAML::Node(YAML::NodeType::Sequence);
        Eigen::Matrix4d T_bundle_cam = SE3Transform::toEigen(pack.T_bundle_sensor);
        for (int i = 0; i < 4; i++) {
            YAML::Node row = YAML::Node(YAML::NodeType::Sequence);
            for (int j = 0; j < 4; j++) {
                row.push_back(T_bundle_cam(i, j));
            }
            T_bundle_cam_node.push_back(row);
        }

        cam_node["intrinsics"] = intrinsics;
        cam_node["resolution"] = resolution;
        if (pack.distortion_type == Distortion::Fisheye) {
            cam_node["distortion_model"] = "equidistant";
        } else if (pack.distortion_type == Distortion::RadTan) {
            cam_node["distortion_model"] = "radtan";
        } else {
            std::cerr << "Trying to serialise unknown distortion type" << std::endl;
            return false;
        }
        cam_node["T_bundle_camera"] = T_bundle_cam_node;
        cam_node["rostopic"] = cam_name;
        cam_node["distortion_coeffs"] = dist_coeffs;
        node["cam" + std::to_string(index++)] = cam_node;
    }
    std::ofstream fout(output_path);
    fout << node;
    return true;
}

std::map<std::string, std::map<unsigned long long, std::shared_ptr<BoardPoseParameterPack>>>
PopulateBoardParameters(const CameraCamera2D3DTargetDetectionData &parsed_alignment_data) {
    std::map<std::string, std::map<unsigned long long, std::shared_ptr<BoardPoseParameterPack>>> board_pose_parameter_packs;

    for (const auto stamp: parsed_alignment_data.unique_timestamps) {
        const auto &data_at_stamp = parsed_alignment_data.observations.at(stamp);
        for (const auto &it: data_at_stamp) {
            Eigen::Affine3d T_bundle_board = it.second.T_sensor_model;
            board_pose_parameter_packs[it.second.sensor_name][stamp] = std::make_shared<BoardPoseParameterPack>();
            SE3Transform::assignToData(T_bundle_board,
                                       board_pose_parameter_packs[it.second.sensor_name][stamp]->T_sensor_board);

            cv::Mat cvObjectPoints, cvImagePoints;
            cv::eigen2cv(it.second.modelpoints3d, cvObjectPoints);
            cv::eigen2cv(it.second.observations2d, cvImagePoints);
        }
    }
    return board_pose_parameter_packs;
}

void GetAllResiduals(ceres::Problem &problem, const std::vector<ceres::ResidualBlockId> &residual_blocks,
                     Eigen::Matrix2Xf &residuals) {

    // Loop over each residual block
    for (const ceres::ResidualBlockId &residual_block: residual_blocks) {
        const ceres::CostFunction *cost_function = problem.GetCostFunctionForResidualBlock(residual_block);
        const ceres::LossFunction *loss_function = problem.GetLossFunctionForResidualBlock(residual_block);

        // Retrieve parameter blocks
        std::vector<double *> parameter_blocks;
        problem.GetParameterBlocksForResidualBlock(residual_block, &parameter_blocks);

        // Allocate space for residuals
        std::vector<double> block_residuals(cost_function->num_residuals());

        // Evaluate the residuals
        cost_function->Evaluate(parameter_blocks.data(), block_residuals.data(), nullptr);

        Eigen::Matrix2Xd residuals2d = Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic>>(block_residuals.data(),
                                                                                            2,
                                                                                            block_residuals.size() / 2);

        // Conservatively resize mat1 to have the required number of columns
        residuals.conservativeResize(residuals.rows(), residuals.cols() + residuals2d.cols());

        // Append mat2 columns to mat1
        residuals.block(0, residuals.cols() - residuals2d.cols(), residuals.rows(),
                        residuals2d.cols()) = residuals2d.cast<float>();

    }
}

PrismPositionDetectionData LoadPrismPositions(std::string json_path) {
    nlohmann::json data;
    PrismPositionDetectionData result;
    {
        std::ifstream f(json_path);
        data = nlohmann::json::parse(f);
    }
    for (const auto& stamp_data: data.items()) {
        const std::string& stamp = stamp_data.key();
        std::vector<double> position_data = stamp_data.value().get<std::vector<double>>();
        Eigen::Map<Eigen::Vector3d> position(position_data.data());
        result[std::stoull(stamp)] = position;
    }
    return result;
}
