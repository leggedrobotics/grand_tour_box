#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <algorithm>
#include <boost/progress.hpp>
#include <filesystem>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <vector>

namespace fs = std::filesystem;

bool fnmatchcase(const std::string& name, const std::string& pattern) {
  return std::regex_match(name, std::regex(pattern));
}

std::pair<int, int> merge_bags_single(const std::vector<std::string>& input_bag, const std::string& output_bag,
                                      const std::string& topics = "*", bool verbose = false) {
  std::vector<std::string> topic_patterns = {topics};

  int total_included_count = 0;
  int total_skipped_count = 0;

  if (verbose) {
    std::cout << "Writing bag file: " << output_bag << std::endl;
    std::cout << "Matching topics against patterns: '" << topics << "'" << std::endl;
  }

  rosbag::Bag out_bag;
  out_bag.open(output_bag, rosbag::bagmode::Write);
  out_bag.setCompression(rosbag::compression::LZ4);

  boost::progress_display progress(input_bag.size());

  for (const auto& ifile : input_bag) {
    std::vector<std::string> matchedtopics;
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
        out_bag.write(msg.getTopic(), msg.getTime(), msg);
        included_count++;
      }

      in_bag.close();
    } catch (const std::exception& e) {
      std::cerr << "Failed to read bag file: " << e.what() << std::endl;
      std::cerr << "Skipping this bag" << std::endl;
    }

    total_included_count += included_count;
    total_skipped_count += skipped_count;

    if (verbose) {
      std::cout << "< Included " << included_count << " messages and skipped " << skipped_count << std::endl;
    }

    ++progress;
  }

  out_bag.close();

  if (verbose) {
    std::cout << "Total: Included " << total_included_count << " messages and skipped " << total_skipped_count << std::endl;
  }

  return {total_included_count, total_skipped_count};
}

int main(int argc, char* argv[]) {
  bool overwrite = false;

  std::string mission_folder = "/mission_data";
  std::vector<fs::path> bag_files;

  for (const auto& entry : fs::recursive_directory_iterator(mission_folder)) {
    if (entry.path().extension() == ".bag") {
      bag_files.push_back(entry.path());
    }
  }

  std::sort(bag_files.begin(), bag_files.end());

  std::cout << "Found files: ";
  for (const auto& file : bag_files) {
    std::cout << file << " ";
  }
  std::cout << std::endl;

  std::map<std::string, std::vector<fs::path>> grouped_files;
  std::regex pattern(R"((.*?_\d+)\.bag$)");

  for (const auto& file : bag_files) {
    std::string filename = file.filename().string();
    std::smatch match;
    if (std::regex_search(filename, match, pattern)) {
      std::string prefix = match[1].str();
      prefix = prefix.substr(0, prefix.find_last_of('_'));
      grouped_files[prefix].push_back(file);
    }
  }

  boost::progress_display progress(grouped_files.size());

  for (const auto& [prefix, files] : grouped_files) {
    fs::path output_bag = fs::path(mission_folder) / (prefix + ".bag");

    if (!overwrite && fs::exists(output_bag)) {
      std::cout << "Output file " << output_bag << " already exists. Skipping merge..." << std::endl;
      continue;
    }

    std::vector<std::string> filelist;
    for (const auto& file : files) {
      filelist.push_back(file.string());
    }

    std::sort(filelist.begin(), filelist.end(), [](const std::string& a, const std::string& b) {
      int num_a = std::stoi(a.substr(a.find_last_of('_') + 1, a.find_last_of('.') - a.find_last_of('_') - 1));
      int num_b = std::stoi(b.substr(b.find_last_of('_') + 1, b.find_last_of('.') - b.find_last_of('_') - 1));
      return num_a < num_b;
    });

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