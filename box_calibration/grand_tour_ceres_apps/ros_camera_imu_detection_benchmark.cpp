// Benchmark: producer-consumer april-grid detection from a rosbag.
// One reader thread walks the bag sequentially; N worker threads each call
// instantiate + cv_bridge + detector in parallel.
// Nothing is written to disk.
//
// Usage:
//   ros_camera_imu_detection_benchmark --bag data.bag [--workers 8]

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <grand_tour_camera_detectors/aprilgridasl.h>
#include <gtboxcalibration/argparsers.h>

static constexpr int    kGridSizeX  = 6;
static constexpr int    kGridSizeY  = 6;
static constexpr double kTagSize    = 0.083;
static constexpr double kTagSpacing = 0.3;
static constexpr size_t kQueueCap   = 128;

// ---------------------------------------------------------------------------
// Bounded blocking queue. pop() returns nullopt when closed and drained.
// ---------------------------------------------------------------------------
template<typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(size_t capacity) : capacity_(capacity) {}

    void push(T item) {
        std::unique_lock lock(mutex_);
        not_full_.wait(lock, [&]{ return queue_.size() < capacity_ || closed_; });
        if (closed_) return;
        queue_.push(std::move(item));
        not_empty_.notify_one();
    }

    std::optional<T> pop() {
        std::unique_lock lock(mutex_);
        not_empty_.wait(lock, [&]{ return !queue_.empty() || closed_; });
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop();
        not_full_.notify_one();
        return item;
    }

    void close() {
        std::unique_lock lock(mutex_);
        closed_ = true;
        not_empty_.notify_all();
        not_full_.notify_all();
    }

private:
    size_t capacity_;
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable not_full_, not_empty_;
    bool closed_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_camera_imu_detection_benchmark");

    argparse::ArgumentParser program("ros_camera_imu_detection_benchmark");
    program.add_argument("--bag")
        .required()
        .help("Input rosbag path.");
    program.add_argument("--workers")
        .default_value(static_cast<int>(std::thread::hardware_concurrency()))
        .scan<'i', int>()
        .help("Number of worker threads (default: hardware concurrency).");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n" << program;
        return 1;
    }

    const std::string bag_path = program.get<std::string>("--bag");
    const int n_workers = program.get<int>("--workers");

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View view(bag);

    std::cout << "Bag:      " << bag_path << "\n"
              << "Messages: " << view.size() << "\n"
              << "Workers:  " << n_workers << "\n\n";

    std::atomic<size_t> n_images{0};
    std::atomic<size_t> n_detections{0};

    BoundedQueue<rosbag::MessageInstance> work_queue(kQueueCap);

    // --- Worker threads: instantiate + decode + detect ---
    std::vector<std::thread> workers;
    workers.reserve(n_workers);

    const auto wall_t0 = std::chrono::steady_clock::now();

    for (int i = 0; i < n_workers; ++i) {
        workers.emplace_back([&]() {
            cameras::GridCalibrationTargetAprilgrid detector(
                kGridSizeX, kGridSizeY, kTagSize, kTagSpacing);

            while (auto item = work_queue.pop()) {
                auto img_msg = item->instantiate<sensor_msgs::Image>();
                if (!img_msg) continue;

                cv::Mat image;
                try {
                    image = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8)->image;
                } catch (const cv_bridge::Exception&) {
                    continue;
                }

                Eigen::MatrixXd corners;
                std::vector<bool> valid;
                detector.computeObservation(image, corners, valid);

                const size_t cur = ++n_images;
                if (cur % 50 == 0) {
                    std::cout << "\r  " << cur << " images / "
                              << n_detections.load() << " detections" << std::flush;
                }

                const size_t found = std::count(valid.begin(), valid.end(), true);
                if (found > 0) ++n_detections;
            }
        });
    }

    // --- Reader: walk bag sequentially, push MessageInstances ---
    for (const auto& m : view) {
        work_queue.push(m);
    }
    work_queue.close();

    for (auto& t : workers) t.join();

    const double wall_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wall_t0).count();

    // bag must outlive workers since MessageInstance holds a ref into it
    bag.close();

    std::cout << "\n\n--- Summary ---\n"
              << "Wall time:        " << wall_s << " s\n"
              << "Workers:          " << n_workers << "\n"
              << "Images processed: " << n_images.load() << "\n"
              << "Frames detected:  " << n_detections.load() << "\n"
              << "Throughput:       " << n_images.load() / wall_s << " images/s\n";

    return 0;
}