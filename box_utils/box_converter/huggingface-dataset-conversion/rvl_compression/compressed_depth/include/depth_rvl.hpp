#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

// Returns CV_32FC1 depth or throws std::runtime_error
cv::Mat decode_depth(const std::string& encoding,
                     const std::string& format_field,
                     const std::vector<uint8_t>& payload);
