#pragma once

#include <filesystem>
#include <memory>
#include <rerun.hpp>

inline std::unique_ptr<rerun::RecordingStream> openRerunRecording(const std::string &name, bool visualize,
                                                                  const std::string &recording_path,
                                                                  const std::string &blueprint_path) {
    if (!visualize && recording_path.empty())
        return nullptr;
    auto recording = std::make_unique<rerun::RecordingStream>(name);
    if (!recording_path.empty())
        recording->save(recording_path).exit_on_failure();
    else
        recording->spawn().exit_on_failure();
    if (!blueprint_path.empty() && std::filesystem::is_regular_file(blueprint_path))
        recording->log_file_from_path(blueprint_path);
    return recording;
}
