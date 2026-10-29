// Regression test for ROSCameraCameraOfflineProgram.
//
// Runs the full offline calibration pipeline against the known bag files
// and checks that the output matches the golden reference YAML within tolerance.
//
// Usage:
//   GT_CALIBRATION_BAG_DIR=/path/to/reference_recording catkin_make run_tests
//
// Skipped only when no recording was configured; an invalid configured path fails.

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>  // for Matrix4d

#include <filesystem>
#include <cstdlib>
#include <unistd.h>
#include <string>
#include <vector>

#include "ros_camera_camera_offline_program.h"
#include "ros_camera_camera_parser.h"

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Optional reference recording, supplied by the test environment
// ---------------------------------------------------------------------------
static const std::string kBagDir =
    std::getenv("GT_CALIBRATION_BAG_DIR") ? std::getenv("GT_CALIBRATION_BAG_DIR") : "";

static const std::string kGoldenYaml = kBagDir + "/output_camera_camera_calibration.yaml";

static const std::vector<std::string> kBagPaths = {
    kBagDir + "/2024-10-31-19-28-44_jetson_hdr_front.bag",
    kBagDir + "/2024-10-31-19-28-44_jetson_hdr_left.bag",
    kBagDir + "/2024-10-31-19-28-44_jetson_hdr_right.bag",
    kBagDir + "/2024-10-31-19-28-44_jetson_zed2i_left.bag",
    kBagDir + "/2024-10-31-19-28-44_jetson_zed2i_right.bag",
    kBagDir + "/2024-10-31-19-28-44_nuc_cam4.bag",
    kBagDir + "/alphasense_front_center.bag",
    kBagDir + "/alphasense_front_left.bag",
    kBagDir + "/alphasense_front_right.bag",
    kBagDir + "/alphasense_right.bag",
};

// ---------------------------------------------------------------------------
// Tolerances
// ---------------------------------------------------------------------------
static constexpr double kIntrinsicsTol = 2;   // pixels
static constexpr double kExtrinsicsTol = 2e-3;   // metres
static constexpr double kDistortionTol = 1e-2;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Parse a 4x4 matrix stored as a sequence-of-rows in YAML into Eigen.
Eigen::Matrix4d yamlToMatrix4d(const YAML::Node& node) {
    Eigen::Matrix4d M;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            M(r, c) = node[r][c].as<double>();
    return M;
}

// ---------------------------------------------------------------------------
// Fixture — program runs once for the whole suite via SetUpTestSuite
// ---------------------------------------------------------------------------

static fs::path gOutputDirectory;
static std::string gOutputYamlPath;

class CalibrationRegressionTest : public ::testing::Test {
protected:
    static YAML::Node output_;
    static YAML::Node golden_;

    void SetUp() override {
        if (kBagDir.empty()) GTEST_SKIP() << "Set GT_CALIBRATION_BAG_DIR to the reference recording";
        ASSERT_TRUE(fs::is_directory(kBagDir)) << kBagDir;
        if (!gOutputDirectory.empty()) {
            ASSERT_TRUE(output_.IsMap() && golden_.IsMap()) << "Calibration setup failed";
            return;
        }
        for (const auto& p : kBagPaths)
            if (!fs::exists(p))
                FAIL() << "Bag not found: " << p;

        char pattern[] = "/tmp/grand_tour_regression_XXXXXX";
        const auto directory = mkdtemp(pattern);
        ASSERT_NE(directory, nullptr);
        gOutputDirectory = directory;
        gOutputYamlPath = (gOutputDirectory / "calibration.yaml").string();

        std::vector<std::string> args_storage = {
            "test_calibration_regression", "--bags"
        };
        for (const auto& b : kBagPaths) args_storage.push_back(b);
        args_storage.insert(args_storage.end(), {"--output_path", gOutputYamlPath});

        std::vector<char*> argv;
        for (auto& s : args_storage) argv.push_back(s.data());
        int argc = static_cast<int>(argv.size());

        ROSCameraCameraParser parser("test_calibration_regression", argc, argv.data());
        ASSERT_TRUE(parser.is_valid) << "Parser invalid — check config files.";

        ROSCameraCameraOfflineProgram program(parser);
        ASSERT_TRUE(program.isValid()) << "Program invalid after construction.";
        ASSERT_TRUE(program.run()) << "program.run() returned false.";
        ASSERT_TRUE(fs::exists(gOutputYamlPath)) << "Output YAML was not written.";

        output_ = YAML::LoadFile(gOutputYamlPath);
        golden_ = YAML::LoadFile(kGoldenYaml);
    }

    static void TearDownTestSuite() {
        if (!gOutputDirectory.empty()) fs::remove_all(gOutputDirectory);
    }
};

YAML::Node CalibrationRegressionTest::output_;
YAML::Node CalibrationRegressionTest::golden_;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_F(CalibrationRegressionTest, SameCameraSet) {
    ASSERT_EQ(output_.size(), golden_.size())
        << "Number of cameras in output differs from golden.";

    for (const auto& kv : golden_) {
        const std::string cam = kv.first.as<std::string>();
        EXPECT_TRUE(output_[cam]) << "Camera '" << cam << "' missing from output.";
    }
}

TEST_F(CalibrationRegressionTest, Intrinsics) {
    for (const auto& kv : golden_) {
        const std::string cam = kv.first.as<std::string>();
        if (!output_[cam]) continue;

        const auto& g_intr = kv.second["intrinsics"];
        const auto& o_intr = output_[cam]["intrinsics"];
        ASSERT_EQ(g_intr.size(), 4u) << cam << ": golden intrinsics size != 4";
        ASSERT_EQ(o_intr.size(), 4u) << cam << ": output intrinsics size != 4";

        for (int i = 0; i < 4; ++i) {
            EXPECT_NEAR(g_intr[i].as<double>(), o_intr[i].as<double>(), kIntrinsicsTol)
                << cam << ": intrinsics[" << i << "] out of tolerance.";
        }
    }
}

TEST_F(CalibrationRegressionTest, DistortionCoeffs) {
    for (const auto& kv : golden_) {
        const std::string cam = kv.first.as<std::string>();
        if (!output_[cam]) continue;

        ASSERT_EQ(kv.second["distortion_coeffs"].size(), output_[cam]["distortion_coeffs"].size());
        for (size_t i = 0; i < kv.second["distortion_coeffs"].size(); ++i)
            EXPECT_NEAR(kv.second["distortion_coeffs"][i].as<double>(),
                        output_[cam]["distortion_coeffs"][i].as<double>(), kDistortionTol);
    }
}

TEST_F(CalibrationRegressionTest, ExtrinsicsTranslation) {
    for (const auto& kv : golden_) {
        const std::string cam = kv.first.as<std::string>();
        if (!output_[cam]) continue;

        const Eigen::Matrix4d T_golden = yamlToMatrix4d(kv.second["T_bundle_camera"]);
        const Eigen::Matrix4d T_output = yamlToMatrix4d(output_[cam]["T_bundle_camera"]);

        for (int r = 0; r < 3; ++r)
            EXPECT_NEAR(T_golden(r, 3), T_output(r, 3), kExtrinsicsTol)
                << cam << ": T_bundle_camera translation[" << r << "]";
    }
}

TEST_F(CalibrationRegressionTest, ExtrinsicsRotation) {
    for (const auto& kv : golden_) {
        const auto camera = kv.first.as<std::string>();
        ASSERT_TRUE(output_[camera]);
        const auto reference = yamlToMatrix4d(kv.second["T_bundle_camera"]);
        const auto actual = yamlToMatrix4d(output_[camera]["T_bundle_camera"]);
        const Eigen::Matrix3d error = reference.block<3,3>(0,0).transpose()*actual.block<3,3>(0,0);
        EXPECT_LT(Eigen::AngleAxisd(error).angle(), 1e-3) << camera; // radians
    }
}

TEST_F(CalibrationRegressionTest, DistortionModel) {
    for (const auto& kv : golden_) {
        const std::string cam = kv.first.as<std::string>();
        if (!output_[cam]) continue;

        EXPECT_EQ(kv.second["distortion_model"].as<std::string>(),
                  output_[cam]["distortion_model"].as<std::string>())
            << cam << ": distortion_model mismatch.";
    }
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_calibration_regression",
              ros::init_options::AnonymousName |
              ros::init_options::NoSigintHandler);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}