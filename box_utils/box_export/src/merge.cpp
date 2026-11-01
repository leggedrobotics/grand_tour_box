#include <box_export/atomic_directory.hpp>
#include <fnmatch.h>
#include <sstream>
// MIT License

// Copyright (c) 2023-2025 ETH Zurich, Robotic Systems Lab.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <algorithm>
#include <cstdint>
#include <boost/progress.hpp>
#include <filesystem>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <vector>

namespace fs = std::filesystem;

std::pair<int, int> merge_bags_single(const std::vector<std::string>& input_bag, const std::string& output_bag,
                                      const std::string& topics = "*", bool verbose = false) {
  std::vector<std::string> topic_patterns;
  std::istringstream patterns(topics);
  for (std::string pattern; patterns >> pattern;) topic_patterns.push_back(pattern);
  if (input_bag.empty() || topic_patterns.empty()) throw std::runtime_error("No inputs or topic patterns");

  int total_included_count = 0;
  int total_skipped_count = 0;

  if (verbose) {
    std::cout << "Writing bag file: " << output_bag << std::endl;
    std::cout << "Matching topics against patterns: '" << topics << "'" << std::endl;
  }

  box_export::AtomicFile output(output_bag);
  rosbag::Bag out_bag;
  out_bag.open(output.path().string(), rosbag::bagmode::Write);
  out_bag.setCompression(rosbag::compression::LZ4);

  boost::progress_display progress(input_bag.size());

  for (const auto& ifile : input_bag) {
    int included_count = 0;
    int skipped_count = 0;

    if (verbose) {
      std::cout << "> Reading bag file: " << ifile << std::endl;
    }

    try {
      rosbag::Bag in_bag;
      in_bag.open(ifile, rosbag::bagmode::Read);
      rosbag::View view(in_bag);

      std::cout << "> Managed to open file: " << ifile << std::endl;

      for (const auto& msg : view) {
        if (std::any_of(topic_patterns.begin(), topic_patterns.end(), [&](const std::string& pattern) {
              return fnmatch(pattern.c_str(), msg.getTopic().c_str(), 0) == 0;
            })) {
          out_bag.write(msg.getTopic(), msg.getTime(), msg);
          included_count++;
        } else ++skipped_count;
      }

      in_bag.close();
    } catch (const std::exception& e) {
      std::cerr << "Failed to read bag file: " << e.what() << std::endl;
      throw;
    }

    total_included_count += included_count;
    total_skipped_count += skipped_count;

    if (verbose) {
      std::cout << "< Included " << included_count << " messages and skipped " << skipped_count << std::endl;
    }

    ++progress;
  }

  out_bag.close();
  if (!total_included_count) throw std::runtime_error("Merge selected no messages");
  output.commit();

  if (verbose) {
    std::cout << "Total: Included " << total_included_count << " messages and skipped " << total_skipped_count << std::endl;
  }

  return {total_included_count, total_skipped_count};
}

static int processMerge(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: merge MISSION_DIRECTORY\n";
    return 1;
  }
  const fs::path mission_folder = fs::absolute(argv[1]).lexically_normal();
  std::vector<fs::path> bag_files;

  for (const auto& entry : fs::recursive_directory_iterator(mission_folder)) {
    if (entry.is_regular_file() && entry.path().extension() == ".bag") {
      bag_files.push_back(entry.path());
    }
  }

  std::sort(bag_files.begin(), bag_files.end());

  std::cout << "Found files: ";
  for (const auto& file : bag_files) {
    std::cout << file << " ";
  }
  std::cout << std::endl;

  struct SplitGroup {
    fs::path directory;
    std::map<std::uint64_t, fs::path> splits;
  };
  std::map<std::string, SplitGroup> grouped_files;
  const std::regex pattern(R"((.+)_([0-9]+)\.bag)");

  // Validate every group before creating any output. Archived copies or another
  // mission's splits must never be silently mixed merely because names match.
  for (const auto& file : bag_files) {
    std::string filename = file.filename().string();
    std::smatch match;
    if (std::regex_match(filename, match, pattern)) {
      const std::string prefix = match[1].str();
      const std::uint64_t index = std::stoull(match[2].str());
      auto& group = grouped_files[prefix];
      if (!group.directory.empty() && group.directory != file.parent_path())
        throw std::runtime_error("Split group '" + prefix + "' spans multiple directories; merge each input directory explicitly: " +
                                 group.directory.string() + " and " + file.parent_path().string());
      group.directory = file.parent_path();
      if (!group.splits.emplace(index, file).second)
        throw std::runtime_error("Duplicate split index " + std::to_string(index) + " for '" + prefix + "'");
    }
  }
  if (grouped_files.empty()) throw std::runtime_error("No split bags found in " + mission_folder.string());

  boost::progress_display progress(grouped_files.size());

  for (const auto& [prefix, group] : grouped_files) {
    const fs::path output_bag = mission_folder / (prefix + ".bag");

    if (fs::exists(output_bag)) {
      std::cout << "Output file " << output_bag << " already exists. Skipping merge..." << std::endl;
      continue;
    }

    std::vector<std::string> filelist;
    // The numeric map orders split 2 before split 10 and treats 0/00 as the
    // same identity, so alternate zero padding cannot duplicate a recording.
    for (const auto& [index, file] : group.splits) {
      filelist.push_back(file.string());
    }

    std::cout << "Merging files: ";
    for (const auto& file : filelist) {
      std::cout << file << " ";
    }
    std::cout << "into " << output_bag << std::endl;

    merge_bags_single(filelist, output_bag.string(), "*", true);
    ++progress;
  }

  return 0;
}

int main(int argc, char* argv[]) {
  try { return processMerge(argc, argv); }
  catch (const std::exception& error) { std::cerr << error.what() << "\n"; return 1; }
}
