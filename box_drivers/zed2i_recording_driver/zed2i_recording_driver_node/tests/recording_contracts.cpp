#include <gtest/gtest.h>

#include <atomic>
#include <fstream>
#include <limits>
#include <thread>
#include <zed2i_recording_driver/recording_session.hpp>
#include <zed2i_recording_driver/sensor_messages.hpp>

namespace {
class FakeCamera : public zed_recording::RecordingBackend {
 public:
  bool active = false, reject_start = false, reject_frame = false, throw_frame = false;
  int starts = 0, stops = 0;
  int pending_frames = 0, grabs = 0;
  uint64_t stamp = 1000000000, step = 66666667;
  std::string enable(const std::string&) override {
    EXPECT_FALSE(active);
    ++starts;
    active = !reject_start;
    return reject_start ? "encoder unavailable" : "";
  }
  zed_recording::Frame grab() override {
    EXPECT_TRUE(active);
    if (throw_frame) throw std::runtime_error("camera disconnected");
    stamp += step;
    ++grabs;
    const bool written = pending_frames == 0;
    if (pending_frames > 0) --pending_frames;
    return {stamp, reject_frame ? "write failed" : "", written};
  }
  void disable() override {
    active = false;
    ++stops;
  }
};

class RecordingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char path[] = "/tmp/zed-recorder-contract-XXXXXX";
    const auto made = ::mkdtemp(path);
    ASSERT_NE(made, nullptr);
    directory = made;
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  std::string output(const std::string& name = "test.svo2") { return (directory / name).string(); }
  std::filesystem::path directory;
  FakeCamera camera;
};

TEST_F(RecordingTest, StartAcknowledgesFirstWrittenFrameAndResetsFrequency) {
  zed_recording::RecordingSession session(camera, 0);
  ASSERT_TRUE(session.start(output()));
  for (int i = 0; i < 12; ++i) session.tick();
  EXPECT_NEAR(session.status().hz, 15., 1e-5);
  EXPECT_TRUE(session.start(output()));
  EXPECT_EQ(camera.starts, 1);
  session.stop();
  EXPECT_FALSE(camera.active);
  EXPECT_DOUBLE_EQ(session.status().hz, 0.);
  ASSERT_TRUE(session.start(output("second.svo2")));
  EXPECT_DOUBLE_EQ(session.status().hz, 0.);
  session.stop();
}

TEST_F(RecordingTest, ExistingFilesAndSymlinksAreNeverPassedToSdk) {
  std::ofstream(output()) << "preserve raw recording";
  std::filesystem::create_symlink(output(), output("link.svo2"));
  zed_recording::RecordingSession session(camera, 0);
  EXPECT_FALSE(session.start(output()));
  EXPECT_FALSE(session.start(output("link.svo2")));
  std::ifstream file(output());
  std::string content((std::istreambuf_iterator<char>(file)), {});
  EXPECT_EQ(content, "preserve raw recording");
  EXPECT_EQ(camera.starts, 0);
}

TEST_F(RecordingTest, EncoderStartupLatencyDoesNotRejectHealthyRecording) {
  camera.pending_frames = 1;
  zed_recording::RecordingSession session(camera, 0);
  EXPECT_TRUE(session.start(output()));
  EXPECT_EQ(camera.grabs, 2);
  EXPECT_TRUE(session.status().active);
  EXPECT_TRUE(session.stop());
}

TEST_F(RecordingTest, EncoderStartupWithoutWrittenFramesFailsWithinBound) {
  camera.pending_frames = 100;
  zed_recording::RecordingSession session(camera, 0);
  EXPECT_FALSE(session.start(output()));
  EXPECT_EQ(camera.grabs, 30);
  EXPECT_FALSE(camera.active);
  EXPECT_EQ(session.status().error, "No SVO frame was written during encoder startup");
}

TEST_F(RecordingTest, FailedWriteAfterStartupIsNotTreatedAsEncoderWarmup) {
  zed_recording::RecordingSession session(camera, 0);
  ASSERT_TRUE(session.start(output()));
  camera.pending_frames = 1;
  session.tick();
  EXPECT_FALSE(session.status().active);
  EXPECT_EQ(session.status().error, "SDK failed to write the SVO frame");
}

TEST_F(RecordingTest, InvalidPathAndFullDiskFailBeforeSdkStart) {
  zed_recording::RecordingSession session(camera, 100, [](const auto&) { return 99; });
  EXPECT_FALSE(session.start("relative.svo2"));
  EXPECT_FALSE(session.start(output("wrong.bag")));
  EXPECT_FALSE(session.start(output()));
  EXPECT_FALSE(std::filesystem::exists(output()));
  EXPECT_EQ(camera.starts, 0);
}

TEST_F(RecordingTest, DifferentDestinationDoesNotInterruptActiveRecording) {
  zed_recording::RecordingSession session(camera, 0);
  ASSERT_TRUE(session.start(output()));
  EXPECT_FALSE(session.start(output("another.svo2")));
  EXPECT_TRUE(session.status().active);
  EXPECT_FALSE(std::filesystem::exists(output("another.svo2")));
  session.stop();
}

TEST_F(RecordingTest, EncoderRefusalAndFirstFrameFailureReturnFailure) {
  zed_recording::RecordingSession session(camera, 0);
  camera.reject_start = true;
  EXPECT_FALSE(session.start(output()));
  EXPECT_FALSE(camera.active);
  camera.reject_start = false;
  camera.reject_frame = true;
  EXPECT_FALSE(session.start(output("second.svo2")));
  EXPECT_FALSE(session.status().active);
  EXPECT_EQ(session.status().error, "write failed");
}

TEST_F(RecordingTest, MidRecordingFailureStopsAndRetainsPartialOutput) {
  zed_recording::RecordingSession session(camera, 0);
  ASSERT_TRUE(session.start(output()));
  camera.reject_frame = true;
  session.tick();
  EXPECT_FALSE(camera.active);
  EXPECT_FALSE(session.status().active);
  EXPECT_TRUE(std::filesystem::exists(output()));
  EXPECT_FALSE(session.stop());
  EXPECT_EQ(session.status().error, "write failed");
}

TEST_F(RecordingTest, CameraExceptionsDuringStartDoNotLeaveRecordingActive) {
  zed_recording::RecordingSession session(camera, 0);
  camera.throw_frame = true;
  EXPECT_FALSE(session.start(output()));
  EXPECT_FALSE(camera.active);
  EXPECT_FALSE(session.status().active);
}

TEST_F(RecordingTest, DuplicateImageTimestampsAreReportedAsFailure) {
  zed_recording::RecordingSession session(camera, 0);
  ASSERT_TRUE(session.start(output()));
  camera.step = 0;
  session.tick();
  EXPECT_FALSE(session.status().active);
  EXPECT_EQ(session.status().error, "Non-increasing ZED image timestamp");
}

TEST_F(RecordingTest, DiskExhaustionDuringCaptureStopsRecording) {
  uintmax_t available = 100;
  zed_recording::RecordingSession session(camera, 100, [&](const auto&) { return available; });
  ASSERT_TRUE(session.start(output()));
  available = 99;
  std::this_thread::sleep_for(std::chrono::milliseconds(1010));
  session.tick();
  EXPECT_FALSE(session.status().active);
  EXPECT_FALSE(camera.active);
}

TEST_F(RecordingTest, ConcurrentServiceCallsAndGrabsRemainSerialized) {
  zed_recording::RecordingSession session(camera, 0);
  std::atomic<bool> run{true};
  std::thread worker([&] {
    while (run) session.tick();
  });
  std::thread stopper([&] {
    for (int i = 0; i < 100; ++i) session.stop();
  });
  for (int i = 0; i < 100; ++i) session.start(output(std::to_string(i) + ".svo2"));
  stopper.join();
  run = false;
  worker.join();
  session.stop();
  EXPECT_FALSE(camera.active);
  EXPECT_FALSE(session.status().active);
}

TEST(SensorMessages, ImuUnitsFullCovarianceAndHostTimestamp) {
  sl::SensorsData data;
  data.imu.timestamp = 1700000000123456789ULL;
  data.imu.angular_velocity = sl::float3(180.f, -90.f, 45.f);
  data.imu.linear_acceleration = sl::float3(1.f, 2.f, 9.81f);
  for (int i = 0; i < 9; ++i) {
    data.imu.angular_velocity_covariance.r[i] = i + 1;
    data.imu.linear_acceleration_covariance.r[i] = i + 10;
  }
  const auto msg = zed_recording::imuMessage(data.imu, "zed2i_imu_link");
  EXPECT_EQ(msg.header.stamp.toNSec(), 1700000000123456789ULL);
  EXPECT_EQ(msg.header.frame_id, "zed2i_imu_link");
  EXPECT_NEAR(msg.angular_velocity.x, 3.141592653589793, 1e-12);
  EXPECT_NEAR(msg.angular_velocity.y, -1.570796326794897, 1e-12);
  EXPECT_NEAR(msg.linear_acceleration.z, 9.81, 1e-6);
  EXPECT_EQ(msg.orientation_covariance[0], -1.);
  for (int i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[i],
                     (i + 1) * zed_recording::kDegreesToRadians * zed_recording::kDegreesToRadians);
    EXPECT_EQ(msg.linear_acceleration_covariance[i], i + 10);
  }
}

TEST(SensorMessages, UnknownCovarianceDoesNotSuppressAvailableMeasurements) {
  sl::SensorsData data;
  data.imu.timestamp = 1700000000123456789ULL;
  data.imu.angular_velocity = sl::float3(1.f, 2.f, 3.f);
  data.imu.linear_acceleration = sl::float3(0.f, 0.f, 9.81f);
  data.imu.angular_velocity_covariance.r[0] = std::numeric_limits<float>::quiet_NaN();
  data.imu.linear_acceleration_covariance.r[0] = -1.f;
  const auto msg = zed_recording::imuMessage(data.imu, "zed2i_imu_link");
  for (int i = 0; i < 9; ++i) {
    EXPECT_EQ(msg.angular_velocity_covariance[i], 0.);
    EXPECT_EQ(msg.linear_acceleration_covariance[i], 0.);
  }
}

TEST(SensorMessages, MagneticFieldAndPressureUseSiUnitsAndIndependentTimestamps) {
  sl::SensorsData data;
  data.magnetometer.timestamp = 1700000000100000000ULL;
  data.barometer.timestamp = 1700000000200000000ULL;
  data.magnetometer.magnetic_field_calibrated = sl::float3(50.f, -10.f, 20.f);
  data.barometer.pressure = 1013.25f;
  const auto mag = zed_recording::magneticMessage(data.magnetometer, "zed2i_mag_link");
  const auto baro = zed_recording::pressureMessage(data.barometer, "zed2i_camera_center");
  EXPECT_NEAR(mag.magnetic_field.x, 50e-6, 1e-15);
  EXPECT_EQ(baro.fluid_pressure, 101325.);
  EXPECT_EQ(mag.header.stamp.toNSec(), 1700000000100000000ULL);
  EXPECT_EQ(baro.header.stamp.toNSec(), 1700000000200000000ULL);
}
}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
