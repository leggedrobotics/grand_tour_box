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
#include <unistd.h>

namespace box_export {
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
    // Linux ROS deployments: NOREPLACE also protects against another writer
    // creating the destination between the initial check and publication.
    if (::syscall(SYS_renameat2, AT_FDCWD, staging_.c_str(), AT_FDCWD,
                  destination_.c_str(), RENAME_NOREPLACE) != 0)
      throw std::runtime_error("Cannot publish dataset: " + std::string(std::strerror(errno)));
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
    // link() publishes only if the destination is still absent. The staging
    // directory's destructor then releases our temporary link.
    std::filesystem::create_hard_link(file_, destination_);
  }
 private:
  std::filesystem::path destination_;
  AtomicDirectory staging_;
  std::filesystem::path file_;
};
}  // namespace box_export
