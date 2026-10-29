#include "ros_camera_camera_offline_program.h"
#include "ros_utils.h"
#include <filesystem>
#include <gtboxcalibration/file_output.h>
#include <gtest/gtest.h>
#include <limits>
#include <std_msgs/String.h>

namespace fs = std::filesystem;
class OfflineCorrectness : public ::testing::Test {
  protected:
    fs::path directory;
    void SetUp() override {
        char pattern[] = "/tmp/grand_tour_offline_test_XXXXXX";
        const auto name = mkdtemp(pattern);
        ASSERT_NE(name, nullptr);
        directory = name;
    }
    void TearDown() override {
        if (!directory.empty())
            fs::remove_all(directory);
    }

    ROSCameraCameraParser parser(const std::string &bag = "detections.bag") {
        std::vector<std::string> args{"test",
                                      "--initial_guess",
                                      (directory / "initial.yaml").string(),
                                      "--rostopic_frameid_mapping",
                                      (directory / "mapping.yaml").string(),
                                      "--bags",
                                      (directory / bag).string(),
                                      "--output_path",
                                      (directory / "result.yaml").string()};
        std::vector<char *> argv;
        for (auto &arg : args)
            argv.push_back(arg.data());
        return ROSCameraCameraParser("test", argv.size(), argv.data());
    }

    void makeFixture(bool observations = true, bool wrong_frame = false, bool duplicate = false,
                     bool fisheye = false) {
        std::map<std::string, CameraParameterPack> packs;
        for (int i = 0; i < 2; ++i) {
            CameraParameterPack camera;
            auto pose = Eigen::Affine3d::Identity();
            pose.translation().x() = .12 * i;
            SE3Transform::assignToData(pose, camera.T_bundle_sensor);
            camera.fxfycxcy[0] = 600;
            camera.fxfycxcy[1] = 610;
            camera.fxfycxcy[2] = 640;
            camera.fxfycxcy[3] = 480;
            camera.width = 1280;
            camera.height = 960;
            camera.distortion_type = fisheye ? Distortion::Fisheye : Distortion::RadTan;
            if (fisheye)
                camera.dist_coeffs[0] = -.04;
            packs["/camera" + std::to_string(i)] = camera;
        }
        ASSERT_TRUE(SerialiseCameraParameters((directory / "initial.yaml").string(), packs, "synthetic"));
        ASSERT_TRUE(
            WriteTextAtomically(directory / "mapping.yaml", "/camera0: camera0\n/camera1: camera1\n"));
        rosbag::Bag bag((directory / "detections.bag").string(), rosbag::bagmode::Write);
        std_msgs::String unrelated;
        unrelated.data = "unrelated topic must be ignored";
        bag.write("/unrelated", ros::Time(1), unrelated);
        if (observations)
            for (int pose_id = 0; pose_id < 30; ++pose_id) {
                // Repeated views at each static board pose exercise temporal matching.
                // Different poses constrain focal length, distortion and the stereo baseline.
                Eigen::Affine3d board = Eigen::Affine3d::Identity();
                board.linear() = (Eigen::AngleAxisd(.35 * std::sin(pose_id * .7), Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(.3 * std::cos(pose_id * .9), Eigen::Vector3d::UnitY()))
                                     .toRotationMatrix();
                board.translation() = Eigen::Vector3d(.3 * std::sin(pose_id), .2 * std::cos(pose_id * .6),
                                                      1.2 + .3 * std::sin(pose_id * .4));
                for (int repeat = 0; repeat < 3; ++repeat)
                    for (int camera = 0; camera < 2; ++camera) {
                        grand_tour_camera_detection_msgs::CameraDetections detection;
                        detection.header.frame_id = wrong_frame ? "wrong" : "camera" + std::to_string(camera);
                        detection.header.stamp = ros::Time(10 + pose_id, 1000000 * repeat);
                        const auto T_camera_board =
                            SE3Transform::toEigenAffine(
                                packs.at("/camera" + std::to_string(camera)).T_bundle_sensor)
                                .inverse() *
                            board;
                        for (int y = 0; y < 10; ++y)
                            for (int x = 0; x < 10; ++x) {
                                Eigen::Vector3d model((x - 4.5) * .045, (y - 4.5) * .045, 0);
                                const Eigen::Vector3d point = T_camera_board * model;
                                geometry_msgs::Point m, p;
                                m.x = model.x();
                                m.y = model.y();
                                m.z = 0;
                                double nx = point.x() / point.z(), ny = point.y() / point.z();
                                if (fisheye) {
                                    const double radius = std::hypot(nx, ny), theta = std::atan(radius);
                                    const double scale =
                                        radius > 1e-12 ? theta * (1 - .04 * theta * theta) / radius : 1.;
                                    nx *= scale;
                                    ny *= scale;
                                }
                                p.x = 600 * nx + 640;
                                p.y = 610 * ny + 480;
                                detection.modelpoint3d.push_back(m);
                                detection.corners2d.push_back(p);
                                detection.cornerids.push_back(y * 10 + x);
                            }
                        const auto topic = "/camera" + std::to_string(camera) + "_corner_detections";
                        bag.write(topic, detection.header.stamp, detection);
                        if (duplicate)
                            bag.write(topic, detection.header.stamp, detection);
                    }
            }
        bag.close();
    }
};

TEST_F(OfflineCorrectness, RecoversKnownStereoCalibrationAndExportsDiagnostics) {
    for (bool fisheye : {false, true}) {
        SCOPED_TRACE(fisheye ? "fisheye" : "radtan");
        makeFixture(true, false, false, fisheye);
        auto arguments = parser();
        ASSERT_TRUE(arguments.is_valid);
        ROSCameraCameraOfflineProgram program(arguments);
        ASSERT_TRUE(program.isValid());
        ASSERT_TRUE(program.run());
        const auto result = YAML::LoadFile((directory / "result.yaml").string());
        ASSERT_EQ(result.size(), 2u);
        for (int camera = 0; camera < 2; ++camera) {
            const auto node = result["cam" + std::to_string(camera)];
            EXPECT_NEAR(node["intrinsics"][0].as<double>(), 600, .1);
            EXPECT_NEAR(node["intrinsics"][1].as<double>(), 610, .1);
            EXPECT_NEAR(node["T_bundle_camera"][0][3].as<double>(), .12 * camera, 1e-4);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    EXPECT_NEAR(node["T_bundle_camera"][r][c].as<double>(), r == c ? 1 : 0, 1e-4);
            ASSERT_TRUE(node["residuals_path"]);
            EXPECT_TRUE(fs::is_regular_file(node["residuals_path"].as<std::string>()));
            ASSERT_TRUE(node["covariances"]);
            for (const auto &sigma : node["covariances"]["fxfycxcy_sigma"])
                EXPECT_TRUE(std::isfinite(sigma.as<double>()));
        }
    }
}

TEST_F(OfflineCorrectness, EmptyBagPreservesExistingCalibration) {
    makeFixture(false);
    ASSERT_TRUE(WriteTextAtomically(directory / "result.yaml", "sentinel"));
    ROSCameraCameraOfflineProgram program(parser());
    EXPECT_FALSE(program.run());
    std::ifstream stream(directory / "result.yaml");
    std::string text;
    stream >> text;
    EXPECT_EQ(text, "sentinel");
}
TEST_F(OfflineCorrectness, FailedSolvePreservesExistingCalibration) {
    struct FailedSolver : ROSCameraCameraOfflineProgram {
        using ROSCameraCameraOfflineProgram::ROSCameraCameraOfflineProgram;
        bool Solve() override { return false; }
    };
    makeFixture();
    ASSERT_TRUE(WriteTextAtomically(directory / "result.yaml", "sentinel"));
    FailedSolver program(parser());
    EXPECT_FALSE(program.run());
    std::ifstream stream(directory / "result.yaml");
    std::string text;
    stream >> text;
    EXPECT_EQ(text, "sentinel");
}
TEST_F(OfflineCorrectness, CorruptBagFailsWithoutOutput) {
    makeFixture(false);
    ASSERT_TRUE(WriteTextAtomically(directory / "corrupt.bag", "invalid bag"));
    ROSCameraCameraOfflineProgram program(parser("corrupt.bag"));
    EXPECT_FALSE(program.run());
    EXPECT_FALSE(fs::exists(directory / "result.yaml"));
}
TEST_F(OfflineCorrectness, WrongFramesAndDuplicateStampsFail) {
    makeFixture(true, true);
    ROSCameraCameraOfflineProgram wrong(parser());
    EXPECT_FALSE(wrong.run());
    makeFixture(true, false, true);
    ROSCameraCameraOfflineProgram duplicate(parser());
    EXPECT_FALSE(duplicate.run());
    EXPECT_FALSE(fs::exists(directory / "result.yaml"));
}
TEST(DetectionValidation, RejectsMalformedArraysAndNonfinitePoints) {
    grand_tour_camera_detection_msgs::CameraDetections message;
    geometry_msgs::Point point;
    message.corners2d.push_back(point);
    EXPECT_THROW(buildObservationFromRosMSG(message), std::invalid_argument);
    message.modelpoint3d.push_back(point);
    message.cornerids.push_back(0);
    message.corners2d[0].x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(buildObservationFromRosMSG(message), std::invalid_argument);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "offline_correctness",
              ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
