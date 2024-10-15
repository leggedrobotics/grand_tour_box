#include <iostream>
#include <string>
#include <deque>
#include <atomic>
#include <chrono>
#include <thread>
#include <sl/Camera.hpp>

class ZedRecordingApp {
private:
    std::deque<sl::Timestamp> timestamps_;
    std::atomic<bool> recording_{false};
    sl::Camera zed_;
    std::string video_filename_;

public:
    ZedRecordingApp() {
        std::cout << "Initializing ZedRecordingApp..." << std::endl;
        openCamera();
    }

    ~ZedRecordingApp() {
        zed_.close();
    }

    void run() {
        while (true) {
            std::cout << "Waiting for recording to start..." << std::endl;
            if (recording_) {
                recordSvo();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        zed_.close();
    }

    sl::RESOLUTION getResolutionFromString(const std::string& resolution) {
        if (resolution == "VGA") {
            return sl::RESOLUTION::VGA;
        } else if (resolution == "HD720") {
            return sl::RESOLUTION::HD720;
        } else if (resolution == "HD1080") {
            return sl::RESOLUTION::HD1080;
        } else if (resolution == "HD2K") {
            return sl::RESOLUTION::HD2K;
        } else {
            std::cerr << "Unknown resolution setting '" << resolution << "'. Defaulting to HD1080." << std::endl;
            return sl::RESOLUTION::HD1080;
        }
    }

    bool validateFramerateForResolution(const std::string& resolution, const int fps) {
        if (resolution == "VGA") {
            return (fps == 15 || fps == 30 || fps == 60 || fps == 100);
        } else if (resolution == "HD720") {
            return (fps == 15 || fps == 30 || fps == 60);
        } else if (resolution == "HD1080") {
            return (fps == 15 || fps == 30);
        } else if (resolution == "HD2K") {
            return (fps == 15);
        } else {
            std::cerr << "Unknown resolution setting '" << resolution << "'. Defaulting to HD1080." << std::endl;
            return (fps == 15 || fps == 30);
        }
    }

    void openCamera() {
        std::string camera_resolution = "HD1080";
        int camera_fps = 30;
        if (!validateFramerateForResolution(camera_resolution, camera_fps)) {
            std::cerr << "Invalid framerate for resolution. Shutting down the application." << std::endl;
            std::exit(EXIT_FAILURE);
        }

        sl::InitParameters init_params;
        init_params.camera_resolution = getResolutionFromString(camera_resolution);
        init_params.depth_mode = sl::DEPTH_MODE::NONE;
        init_params.camera_fps = camera_fps;
        init_params.input.setFromCameraID(0);

        sl::ERROR_CODE err = zed_.open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            std::cerr << "ZED camera cannot be opened, error: " << sl::toString(err) << std::endl;
            std::exit(EXIT_FAILURE);
        }
        std::cout << "ZED camera opened successfully! Resolution: " << camera_resolution << ", FPS: " << camera_fps << std::endl;
    }

    void startRecording(const std::string& filename) {
        recording_ = true;
        video_filename_ = filename;
        std::cout << "Recording started. Filename: " << video_filename_ << std::endl;
    }

    void stopRecording() {
        recording_ = false;
        std::cout << "Recording stopped." << std::endl;
    }

 private:
    void recordSvo() {
        sl::RecordingParameters recording_params;
        recording_params.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
        recording_params.video_filename = sl::String(video_filename_.c_str());

        auto err = zed_.enableRecording(recording_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            std::cerr << "Failed to start recording, error: " << sl::toString(err) << ". Shutting down the application." << std::endl;
            std::exit(EXIT_FAILURE);
        }

        while (recording_) {
            if (zed_.grab() == sl::ERROR_CODE::SUCCESS) {
                auto grab_ts = zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE);
                timestamps_.push_back(grab_ts);
                if (timestamps_.size() > 10) {
                    timestamps_.pop_front();
                    auto window_size = 10;
                    auto time_span_nanos = timestamps_.back().getNanoseconds() - timestamps_.front().getNanoseconds();
                    double avg_hz = 1 / ((time_span_nanos / 1e9) / (window_size - 1));  // Calculate Hz
                    std::cout << "Publishing recording frequency: " << avg_hz << " Hz" << std::endl;
                }
            }
        }
        zed_.disableRecording();
        std::cout << "Recording ended." << std::endl;
    }
};

int main(int argc, char** argv) {
    ZedRecordingApp app;
    if (argc > 1 && std::string(argv[1]) == "start") {
        if (argc > 2) {
            app.startRecording(argv[2]);
        } else {
            std::cerr << "No filename provided for recording." << std::endl;
            return 1;
        }
    }
    app.run();
    return 0;
}