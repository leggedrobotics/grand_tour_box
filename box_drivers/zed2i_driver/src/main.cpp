#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <atomic>
#include <Mutex.h>
#include <ros/console.h>
#include <zed_recording_node/start_recording_svo.h>

std::atomic<bool> start_recording(false);
Mutex mutex;

bool startRecordingService(zed_recording_node::start_recording_svo::Request &req,
                           zed_recording_node::start_recording_svo::Response &res)
{
    start_recording = req.start_recording;
    res.success = true;
    return true;
}

void publishRecordingHz(ros::Publisher &pub, std::atomic<bool> &running, std::deque<std::chrono::high_resolution_clock::time_point> &timestamps)
{
    while(running) {
        if(!start_recording) continue
        std_msgs::Float32 msg;
        std::chrono::duration<double> time_span = 0.0;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (timestamps.size() == 10) {
                time_span = std::chrono::duration_cast<std::chrono::milliseconds>(timestamps.back() - timestamps.front()).count();
            }
        }
        if (time_span == 0.0) {
            continue;
        }
        double avg_hz = 9000.0 / time_span;  // 9000 ms in 10 intervals, converting to seconds for Hz
        msg.data = avg_hz;
        pub.publish(msg);
        ros::Duration(1.0).sleep();  // Publish at 1 Hz
    }
}

void recordSvo(sl::Camera& zed, std::atomic<bool> &running, std::deque<std::chrono::high_resolution_clock::time_point> &timestamps) {

    while (!start_recording) {
        ros::Duration(0.01).sleep()
    }

	RecordingParameters recording_params;
	recording_params.compression_mode = SVO_COMPRESSION_MODE::H264;
    recording_params.video_filename = String(svo_name.c_str());

	auto cam_infos = zed.getCameraInformation();
	
	auto err = zed.enableRecording(recording_params);
	if (err != ERROR_CODE::SUCCESS){
		std::cout << "ZED [" << id << "] can not record, " << err << std::endl;
    }

	int fps = cam_infos.camera_configuration.fps;
	std::cout << "ZED[" << id << "]" << " Start recording "<< cam_infos.camera_configuration.resolution.width<<"@"<< fps <<"\n";
	
    sl::RecordingStatus state;
    while (running) {
        
        auto start_grab = std::chrono::high_resolution_clock::now();
        
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            auto end_grab = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(mutex);
                timestamps.push_back(end_grab);
                if (timestamps.size() > 10) {
                    timestamps.pop_front();
                }
            }
            state = zed.getRecordingStatus();
            if (!state.status) {
                // TODO: Handle recording failure
                continue
            }
        }
    }
	zed.disableRecording();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed_recording_node");
    ros::NodeHandle nh;
    
    // Service to start recording
    ros::ServiceServer service = nh.advertiseService("start_recording", startRecordingService);

    // Set up thread pool.
    std::vector<std::thread> pool(/*Zed Recorder and Hz Publisher =*/2);
    std::atomic<bool> run_threads(true);

    // Timestamps for calculating Hz, shared between threads.
    std::deque<std::chrono::high_resolution_clock::time_point> timestamps;

    // Publisher for recording Hz
    ros::Publisher hz_pub = nh.advertise<std_msgs::Float32>("recording_hz", 10);

    pool[0] = std::thread(publishRecordingHz, std::ref(hz_pub), std::ref(run_threads));

    // Set configuration parameters for the ZED
    // TODO(kappi): Set these from params where appropriate.
    InitParameters init_params;	
    init_params.camera_resolution = RESOLUTION::HD1080;
    init_params.depth_mode = DEPTH_MODE::NONE;
    init_params.camera_fps = 30;
    init_params.input.setFromCameraID(0);

    // Create a ZED camera
    Camera zed;
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        // TODO(kappi): Exit gracefully.
        std::cout <<"ZED ["<<z<<"] can not be opened, "<< err<< std::endl;
    }
    if zed.isOpened() {
        pool[1] = std::thread(recordSvo, std::ref(zed), std::ref(run_threads));
    }

    ros::spin();  // Handle ROS events

    run_threads = false;
    for (auto &t : pool) {
        if (t.joinable()) {
            t.join();
        }
    }

	zeds.close();

    return 0;
}
