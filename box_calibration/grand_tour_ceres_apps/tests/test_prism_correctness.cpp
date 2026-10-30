#include <gtest/gtest.h>
#include <filesystem>
#include <gtboxcalibration/ceresprograms.h>
#include <gtboxcalibration/prismpositionresiduals.h>

TEST(PrismCorrectness, FractionalMedianAndInvalidInput) {
    EXPECT_DOUBLE_EQ(prismResidualMedian({0.001, 0.003, 0.004, 0.002}), 0.0025);
    EXPECT_DOUBLE_EQ(prismResidualMedian({0., 0., 0.}), 0.);
    EXPECT_THROW(prismResidualMedian({}), std::invalid_argument);
    EXPECT_THROW(prismResidualMedian({NAN}), std::invalid_argument);
}

TEST(PrismCorrectness, InterpolationEdgesAndGaps) {
    PrismPositionDetectionData data{{100, Eigen::Vector3d::Zero()}, {200, Eigen::Vector3d::Zero()}};
    EXPECT_TRUE(prismTimeBounds(data, 100, 0));
    EXPECT_TRUE(prismTimeBounds(data, 150, 0));
    EXPECT_TRUE(prismTimeBounds(data, 200, 0));
    EXPECT_FALSE(prismTimeBounds(data, 99, 0));
    EXPECT_FALSE(prismTimeBounds(data, 201, 0));
    EXPECT_FALSE(prismTimeBounds(data, 100, -1));
    data[200000000] = Eigen::Vector3d::Zero();
    EXPECT_FALSE(prismTimeBounds(data, 100000000, 0));
}

TEST(PrismCorrectness, ThreeMillimetreSigmaAndUnsupportedTime) {
    PrismPositionDetectionData data{{100, {0, 0.003, 0}}, {200, {0, 0.003, 0}}};
    PrismInCam0InBoardInTotalStationConsistencyError error(Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(), 150, data);
    double pose[7];
    SE3Transform::assignToData(Eigen::Affine3d::Identity(), pose);
    double lever[3] = {}, offset = 0, residual[3];
    ASSERT_TRUE(error(pose, lever, &offset, residual));
    EXPECT_DOUBLE_EQ(residual[0], 0);
    EXPECT_NEAR(residual[1], 1., 1e-12);
    offset = -1;
    EXPECT_FALSE(error(pose, lever, &offset, residual));
}

static void populateSynthetic(CameraPrismProgram& program, bool outliers, bool degenerate = false) {
    program.cam0_name = "camera";
    program.T_bundle_cam["camera"] = Eigen::Affine3d::Identity();
    SE3Transform::assignToData(Eigen::Affine3d::Identity(), program.prism_board_in_total_station_params.T_totalstation_board);
    const Eigen::Vector3d lever(0.12, -0.03, 0.07);
    for (unsigned i = 0; i < 80; ++i) {
        const auto stamp = 1000000000ull + i * 50000000ull;
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        const double t = i * 0.13;
        if (!degenerate) {
            pose.linear() = (Eigen::AngleAxisd(0.4 * sin(t), Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.3 * cos(0.7*t), Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0.2 * sin(1.3*t), Eigen::Vector3d::UnitZ())).toRotationMatrix();
            pose.translation() = Eigen::Vector3d(0.5*sin(t), 0.3*cos(0.9*t), 2+0.1*sin(1.2*t));
        }
        program.prism_detections[stamp] = pose * lever;
        if (outliers && i >= 76) pose.translation().y() += 0.15;
        auto& observation = program.camera_detections.observations[stamp]["camera"];
        observation.T_sensor_model = pose.inverse();
        program.camera_detections.unique_timestamps.insert(stamp);
    }
}

TEST(PrismCorrectness, ExactRecoveryRetainsZeroResiduals) {
    CameraPrismProgram program;
    populateSynthetic(program, false);
    ASSERT_TRUE(program.PopulateProblem());
    ASSERT_TRUE(program.Solve());
    EXPECT_EQ(program.problem_->getProblem().NumResidualBlocks(), 80);
    EXPECT_EQ(program.camera_detections.unique_timestamps.size(), 80u);
    const Eigen::Map<const Eigen::Vector3d> lever(program.prism_board_in_total_station_params.t_cam0_prism);
    EXPECT_LT((lever - Eigen::Vector3d(0.12, -0.03, 0.07)).norm(), 1e-7);
}

TEST(PrismCorrectness, SecondSolveRemovesOutlierBlocks) {
    CameraPrismProgram program;
    populateSynthetic(program, true);
    ASSERT_TRUE(program.PopulateProblem());
    ASSERT_TRUE(program.Solve());
    EXPECT_LT(program.problem_->getProblem().NumResidualBlocks(), 80);
    EXPECT_EQ(program.problem_->getProblem().NumResidualBlocks(), program.camera_detections.unique_timestamps.size());
    const Eigen::Map<const Eigen::Vector3d> lever(program.prism_board_in_total_station_params.t_cam0_prism);
    EXPECT_LT((lever - Eigen::Vector3d(0.12, -0.03, 0.07)).norm(), 1e-6);
}

TEST(PrismCorrectness, RejectsEmptyAndUnobservableCalibration) {
    CameraPrismProgram empty;
    EXPECT_FALSE(empty.PopulateProblem());
    EXPECT_FALSE(empty.Solve());
    EXPECT_FALSE(empty.WriteOutputParameters());
    CameraPrismProgram degenerate;
    populateSynthetic(degenerate, false, true);
    ASSERT_TRUE(degenerate.PopulateProblem());
    EXPECT_FALSE(degenerate.Solve());
}

TEST(PrismCorrectness, RejectsMalformedCalibrationArraysBeforeReadingThem) {
    const char* text = "camera: {rostopic: /camera, intrinsics: [600, 610, 640, 480], "
                       "distortion_model: radtan, distortion_coeffs: [0, 0, 0, 0], resolution: [1280, 960], "
                       "T_bundle_camera: [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]}";
    auto data = YAML::Load(text);
    EXPECT_NO_THROW(FetchIntrinsicsAsCameraPackFromYaml(data));
    EXPECT_NO_THROW(FetchExtrinsicsFromYaml(data));
    data["camera"]["intrinsics"] = std::vector<double>{600};
    EXPECT_THROW(FetchIntrinsicsAsCameraPackFromYaml(data), std::invalid_argument);
    data = YAML::Load(text);
    data["camera"]["resolution"] = std::vector<int>{1280};
    EXPECT_THROW(FetchIntrinsicsAsCameraPackFromYaml(data), std::invalid_argument);
    data = YAML::Load(text);
    data["camera"]["distortion_model"] = "unsupported";
    EXPECT_THROW(FetchIntrinsicsAsCameraPackFromYaml(data), std::invalid_argument);
    data["camera"]["T_bundle_camera"] = std::vector<double>{1, 2, 3};
    EXPECT_THROW(FetchExtrinsicsFromYaml(data), std::invalid_argument);
    data = YAML::Load(text);
    data["camera"]["T_bundle_camera"][0][0] = 2.;
    EXPECT_THROW(FetchExtrinsicsFromYaml(data), std::invalid_argument);
}

TEST(PrismCorrectness, ShippedCameraConfigurationsRemainSupported) {
    const auto config = std::filesystem::path(__FILE__).parent_path().parent_path() / "config";
    for (const char* name : {"initial_guess.yaml", "initial_guess_color.yaml", "initial_guess_hdr_front_right_swapped.yaml"}) {
        const auto data = YAML::LoadFile((config / name).string());
        EXPECT_EQ(FetchIntrinsicsAsCameraPackFromYaml(data).size(), 10u);
        EXPECT_EQ(FetchExtrinsicsFromYaml(data).size(), 10u);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
