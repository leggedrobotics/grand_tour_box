#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>

namespace zed_recording {

struct Frame {
  uint64_t timestamp_ns = 0;
  std::string error;
  bool written = true;
};

class RecordingBackend {
 public:
  virtual ~RecordingBackend() = default;
  virtual std::string enable(const std::string& path) = 0;
  virtual Frame grab() = 0;
  virtual void disable() = 0;
};

struct RecordingStatus {
  bool active = false;
  std::string filename;
  std::string error;
  double hz = 0.0;
};

// One lock covers SDK recording calls as well as state. Start, stop and grab can
// never overlap, even if a caller later changes the ROS spinner configuration.
// The worker lives for the node's lifetime; it is not replaced on each recording.
class RecordingSession {
 public:
  using SpaceCheck = std::function<uintmax_t(const std::filesystem::path&)>;
  RecordingSession(
      RecordingBackend& backend, uintmax_t minimum_free_bytes,
      SpaceCheck space = [](const std::filesystem::path& p) { return std::filesystem::space(p).available; })
      : backend_(backend), minimum_free_bytes_(minimum_free_bytes), space_(space) {}

  bool start(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool started_here = false;
    try {
      const std::filesystem::path requested(filename);
      if (!requested.is_absolute() || requested.filename().empty() ||
          (requested.extension() != ".svo" && requested.extension() != ".svo2")) {
        throw std::runtime_error("An absolute .svo or .svo2 output path is required");
      }
      // Canonicalize only the existing parent: the output must not exist yet.
      const auto path = std::filesystem::canonical(requested.parent_path()) / requested.filename();
      if (status_.active) {
        if (path.string() == status_.filename) return true;
        throw std::runtime_error("Already recording a different file; stop it first");
      }
      checkSpace(path);
      // The SDK overwrites its destination. Reserve a new name atomically first;
      // an existing file, directory or symlink is never passed to enableRecording.
      const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0644);
      if (fd < 0) throw std::system_error(errno, std::generic_category(), "Cannot reserve SVO output");
      ::close(fd);
      status_ = {false, path.string(), "", 0.0};
      timestamps_.clear();
      started_here = true;
      status_.error = backend_.enable(status_.filename);
      if (!status_.error.empty()) {
        backend_.disable();
        return false;
      }
      status_.active = true;
      last_space_check_ = std::chrono::steady_clock::now();
      // H264 has encoder latency: the first successful grab can report no
      // written frame until the next grab. Bound startup to 30 captured frames
      // (2 seconds at the shipped 15 Hz), and ACK only after a confirmed write.
      for (int attempt = 0; attempt < 30 && status_.active; ++attempt)
        if (captureFrame(true)) return true;
      if (status_.active) fail("No SVO frame was written during encoder startup");
      return false;
    } catch (const std::exception& error) {
      if (started_here)
        fail(error.what());
      else
        status_.error = error.what();
      return false;
    }
  }

  bool stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_.active) backend_.disable();
    status_.active = false;
    status_.hz = 0.0;
    // Keep a recording failure visible after stop, until the next successful start.
    return status_.error.empty();
  }

  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.active) return;
    try {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_space_check_ >= std::chrono::seconds(1)) {
        checkSpace(status_.filename);
        last_space_check_ = now;
      }
      captureFrame();
    } catch (const std::exception& error) {
      fail(error.what());
    }
  }

  RecordingStatus status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

 private:
  void checkSpace(const std::filesystem::path& path) const {
    if (space_(path.parent_path()) < minimum_free_bytes_)
      throw std::runtime_error("Insufficient free space for SVO recording");
  }

  void fail(const std::string& error) {
    backend_.disable();
    status_.active = false;
    status_.hz = 0.0;
    status_.error = error;
    // Preserve partial recordings for inspection; never delete or reuse them.
  }

  bool captureFrame(bool starting = false) {
    const auto frame = backend_.grab();
    if (!frame.error.empty()) {
      fail(frame.error);
      return false;
    }
    if (frame.timestamp_ns == 0 || (!timestamps_.empty() && frame.timestamp_ns <= timestamps_.back())) {
      fail("Non-increasing ZED image timestamp");
      return false;
    }
    if (!frame.written && !starting) {
      fail("SDK failed to write the SVO frame");
      return false;
    }
    timestamps_.push_back(frame.timestamp_ns);
    if (timestamps_.size() > 10) timestamps_.pop_front();
    if (timestamps_.size() > 1)
      status_.hz = (timestamps_.size() - 1) * 1e9 / (timestamps_.back() - timestamps_.front());
    return frame.written;
  }

  RecordingBackend& backend_;
  uintmax_t minimum_free_bytes_;
  SpaceCheck space_;
  mutable std::mutex mutex_;
  RecordingStatus status_;
  std::deque<uint64_t> timestamps_;
  std::chrono::steady_clock::time_point last_space_check_;
};
}  // namespace zed_recording
