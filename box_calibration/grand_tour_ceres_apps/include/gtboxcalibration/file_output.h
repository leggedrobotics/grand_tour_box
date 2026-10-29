#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

inline bool WriteTextAtomically(const std::filesystem::path &destination, const std::string &text) {
    std::string pattern = destination.string() + ".tmp.XXXXXX";
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const int fd = mkstemp(buffer.data());
    if (fd < 0)
        return false;
    close(fd);
    const std::filesystem::path temporary(buffer.data());
    try {
        std::ofstream stream;
        stream.exceptions(std::ios::failbit | std::ios::badbit);
        stream.open(temporary);
        stream << text;
        stream.close();
        std::filesystem::rename(temporary, destination);
        return true;
    } catch (const std::exception &error) {
        std::cerr << "Failed to write " << destination << ": " << error.what() << '\n';
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return false;
    }
}
