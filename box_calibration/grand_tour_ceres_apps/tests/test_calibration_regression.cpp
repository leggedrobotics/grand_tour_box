// Regression test for ROSCameraCameraOfflineProgram.
//
// Runs the full offline calibration pipeline against the known bag files
// and checks that the output matches the golden reference YAML within tolerance.
//
// Usage:
//   catkin_make run_tests  (slow — needs bags on cloudster31)
//
// The test is skipped automatically if the bag directory is not mounted.

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>  // for Matrix4d

#include <filesystem>
#include <string>
#include <vector>

#include "ros_camera_camera_offline_program.h"
#include "ros_camera_camera_parser.h"

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Paths — edit if the mount point changes
// ---------------------------------------------------------------------------
static const std::string kBagDir =
    "/media/fu/cloudster31/gt_handover/2024-10-31-19-28-44_camera_calibration_corrected";

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

static std::string gOutputYamlPath =
    (fs::temp_directory_path() / "calibration_regression_output.yaml").string();

class CalibrationRegressionTest : public ::testing::Test {
protected:
    static YAML::Node output_;
    static YAML::Node golden_;

    static void SetUpTestSuite() {
        if (!fs::exists(kBagDir))
            GTEST_SKIP() << "Bag directory not mounted: " << kBagDir;
        for (const auto& p : kBagPaths)
            if (!fs::exists(p))
                GTEST_SKIP() << "Bag not found: " << p;

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
        if (fs::exists(gOutputYamlPath))
            fs::remove(gOutputYamlPath);
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

        EXPECT_NEAR(kv.second["distortion_coeffs"][0].as<double>(),
                    output_[cam]["distortion_coeffs"][0].as<double>(),
                    kDistortionTol)
            << cam << ": distortion_coeffs[0] out of tolerance.";
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