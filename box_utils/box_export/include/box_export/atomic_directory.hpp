#pragma once

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdlib>
#include <fcntl.h>
#include <linux/fs.h>
#include <sys/syscall.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace box_export {
// Match box_auto.utils.atomic_file's parent-directory locking protocol when a
// filesystem cannot atomically refuse replacement. The final rename publishes
// a complete output; non-cooperating writers need separate coordination there.
inline void publishNoReplace(const std::filesystem::path& source, const std::filesystem::path& destination) {
  if (::syscall(SYS_renameat2, AT_FDCWD, source.c_str(), AT_FDCWD, destination.c_str(), RENAME_NOREPLACE) == 0) return;
  int error = errno;
  if (error != ENOSYS && error != EINVAL && error != EOPNOTSUPP)
    throw std::runtime_error("Cannot publish output: " + std::string(std::strerror(error)));

  if (std::filesystem::is_regular_file(source)) {
    if (::link(source.c_str(), destination.c_str()) == 0) return;
    error = errno;
    if (error != ENOSYS && error != EINVAL && error != EOPNOTSUPP && error != EPERM)
      throw std::runtime_error("Cannot publish output: " + std::string(std::strerror(error)));
  }

  const int parent = ::open(destination.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
  if (parent < 0) throw std::runtime_error("Cannot open output directory: " + std::string(std::strerror(errno)));
  // Close on every path: flock is released with the descriptor.
  struct DirectoryLock {
    int fd;
    ~DirectoryLock() { ::close(fd); }
  } lock{parent};
  if (::flock(parent, LOCK_EX) != 0)
    throw std::runtime_error("Cannot lock output directory: " + std::string(std::strerror(errno)));
  struct stat existing;
  if (::lstat(destination.c_str(), &existing) == 0)
    throw std::runtime_error("Output already exists: " + destination.string());
  if (errno != ENOENT)
    throw std::runtime_error("Cannot inspect output: " + std::string(std::strerror(errno)));
  if (::rename(source.c_str(), destination.c_str()) != 0)
    throw std::runtime_error("Cannot publish output: " + std::string(std::strerror(errno)));
}

// Publish a complete dataset (images, poses and metadata) as one directory.
// Failure removes only our staging directory; an existing dataset is never erased.
class AtomicDirectory {
 public:
  explicit AtomicDirectory(const std::filesystem::path& destination)
      : destination_(std::filesystem::absolute(destination).lexically_normal()) {
    if (destination.empty() || std::filesystem::exists(destination_))
      throw std::runtime_error("Choose a new output directory: " + destination_.string());
    std::filesystem::create_directories(destination_.parent_path());
    std::string pattern = (destination_.parent_path() / ("." + destination_.filename().string() + ".XXXXXX")).string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    char* result = ::mkdtemp(buffer.data());
    if (!result) throw std::runtime_error("Cannot create output staging directory: " + std::string(std::strerror(errno)));
    staging_ = result;
  }
  AtomicDirectory(const AtomicDirectory&) = delete;
  AtomicDirectory& operator=(const AtomicDirectory&) = delete;
  ~AtomicDirectory() {
    if (!staging_.empty()) {
      std::error_code error;
      std::filesystem::remove_all(staging_, error);
    }
  }
  const std::filesystem::path& path() const { return staging_; }
  void commit() {
    publishNoReplace(staging_, destination_);
    staging_.clear();
  }
 private:
  std::filesystem::path destination_, staging_;
};

class AtomicFile {
 public:
  explicit AtomicFile(const std::filesystem::path& destination)
      : destination_(std::filesystem::absolute(destination)), staging_(destination),
        file_(staging_.path() / "output.bag") {}
  const std::filesystem::path& path() const { return file_; }
  void commit() {
    publishNoReplace(file_, destination_);
  }
 private:
  std::filesystem::path destination_;
  AtomicDirectory staging_;
  std::filesystem::path file_;
};
}  // namespace box_export
