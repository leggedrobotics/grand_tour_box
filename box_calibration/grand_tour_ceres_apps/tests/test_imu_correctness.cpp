#include <filesystem>
#include <gtboxcalibration/ceresprograms.h>
#include <gtboxcalibration/file_output.h>
#include <gtboxcalibration/imuresiduals.h>
#include <gtest/gtest.h>

TEST(ImuCalibration, EmptyAndUnbracketedDataAreRejected) {
    CameraIMUProgram program;
    EXPECT_FALSE(program.PopulateProblem());
    program.camera_detections.unique_timestamps = {1000000000ULL, 2000000000ULL};
    EXPECT_FALSE(program.PopulateProblem());
    program.imu_observations = {{Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81), 3.0},
                                {Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81), 4.0}};
    EXPECT_FALSE(program.PopulateProblem());
}

TEST(ImuCalibration, StationaryAndClippedIntervalsMatchAnalyticalMotion) {
    // Piecewise acceleration: 2 m/s^2 until t=.25, then 4 m/s^2 until t=.75.
    // Camera interval [.1,.75] starts between IMU samples.
    std::vector<IMUObservation> samples = {{Eigen::Vector3d::Zero(), Eigen::Vector3d(2, 0, 9.81), 0.0},
                                           {Eigen::Vector3d::Zero(), Eigen::Vector3d(4, 0, 9.81), 0.25}};
    Eigen::Matrix3Xd points(3, 1);
    points << 0.1, 0.2, 2.0;
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    const double velocity = 2 * .15 + 4 * .5;
    const double position = .5 * 2 * .15 * .15 + (2 * .15) * .5 + .5 * 4 * .5 * .5;
    pose.translation().x() = position;
    RelativeIMUBoardPointError error(samples, .1, .75, points, pose.inverse() * points, true);
    double identity[7], next[7];
    SE3Transform::assignToData(Eigen::Affine3d::Identity(), identity);
    SE3Transform::assignToData(pose, next);
    double gravity[3]{0, 0, -1}, zero[3]{}, next_velocity[3]{velocity, 0, 0};
    double residual[18];
    ASSERT_TRUE(
        error(gravity, identity, zero, zero, zero, identity, next, next_velocity, zero, zero, residual));
    for (double value : residual)
        EXPECT_NEAR(value, 0, 1e-12);

    samples = {{Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81), 0.0}};
    RelativeIMUBoardPointError stationary(samples, 0, 1, points, points, true);
    ASSERT_TRUE(
        stationary(gravity, identity, zero, zero, zero, identity, identity, zero, zero, zero, residual));
    for (double value : residual)
        EXPECT_NEAR(value, 0, 1e-12);
}

TEST(ImuCalibration, HandEyeUsesForwardRelativeRotation) {
    Eigen::Affine3d extrinsic = Eigen::Affine3d::Identity();
    extrinsic.linear() = Eigen::AngleAxisd(.4, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    const Eigen::Matrix3d relative = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d camera_relative =
        extrinsic.rotation() * relative * extrinsic.rotation().transpose();
    RelativePoseExtrinsicError error(Eigen::Matrix3d::Identity(), relative, camera_relative);
    double parameters[7], residual[3];
    SE3Transform::assignToData(extrinsic, parameters);
    ASSERT_TRUE(error(parameters, residual));
    for (double value : residual)
        EXPECT_NEAR(value, 0, 1e-12);
}

TEST(ImuCalibration, PresolveRecoversKnownCameraImuRotation) {
    CameraIMUProgram program;
    CameraParameterPack camera;
    SE3Transform::assignToData(Eigen::Affine3d::Identity(), camera.T_bundle_sensor);
    program.camera_packs["camera"] = camera;
    SE3Transform::assignToData(Eigen::Affine3d::Identity(), program.T_camera_bundle_imu);
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(.3, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
    for (int step = 0; step <= 300; ++step) {
        const double time = 1 + step * .01;
        const Eigen::Vector3d gyro(.3 * std::sin(time), .4 * std::cos(time * 1.7), .2 * std::sin(time * .7));
        program.imu_observations.push_back({gyro, Eigen::Vector3d(0, 0, 9.81), time});
        if (step % 10 == 0) {
            const auto stamp = 1000000000ULL + step * 10000000ULL;
            Observations2dModelPoints3dPointIDsPose3dSensorName observation;
            observation.T_sensor_model = Eigen::Affine3d::Identity();
            observation.T_sensor_model.linear() = expected * orientation.transpose();
            program.camera_detections.unique_timestamps.insert(stamp);
            program.camera_detections.observations[stamp]["camera"] = observation;
        }
        orientation *= Sophus::SO3d::exp(gyro * .01).matrix();
    }
    ASSERT_TRUE(program.PreSolveExtrinsic());
    const auto actual = SE3Transform::toEigenAffine(program.T_camera_bundle_imu).rotation();
    EXPECT_LT(Eigen::AngleAxisd(expected.transpose() * actual).angle(), 1e-5);
}

TEST(CalibrationOutput, FailedWriteDoesNotTruncateExistingFile) {
    char pattern[] = "/tmp/grand_tour_output_test_XXXXXX";
    const auto name = mkdtemp(pattern);
    ASSERT_NE(name, nullptr);
    const std::filesystem::path directory(name);
    EXPECT_TRUE(WriteTextAtomically(directory / "result.yaml", "sentinel"));
    EXPECT_FALSE(WriteTextAtomically(directory / "missing" / "result.yaml", "new"));
    std::ifstream stream(directory / "result.yaml");
    std::string text;
    stream >> text;
    EXPECT_EQ(text, "sentinel");
    std::filesystem::remove_all(directory);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
