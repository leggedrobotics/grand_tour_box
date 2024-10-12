#include <iostream>
#include <gtboxcalibration/json.h>
#include <gtboxcalibration/argparse.h>
#include <gtboxcalibration/detectiongraphutils.h>

#include <string>
#include <Eigen/Dense>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/graphviz.hpp>

#include <map>

using namespace nlohmann;
using namespace boost;
using namespace gt_box::graph_utils;

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("program_name");
    program.add_argument("-p", "--poses")
            .required()
            .help("specify the output file.");
    program.add_argument("-t", "--time_tolerance_secs")
            .required()
            .help("threshold time for considering corresponding detections")
            .default_value(0.02)
            .scan<'g', double>();
    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }
    auto input = program.get<std::string>("p");
    auto time_tolerance_secs = program.get<double>("t");
    std::cout << input << std::endl;
    std::cout << "Using time tolerance of " << time_tolerance_secs << " seconds" << std::endl;
    std::ifstream f(input);
    json data = json::parse(f);
    std::vector<unsigned long long> stamps = data["stamps"];
    std::set<unsigned long long> unique_timestamps(stamps.begin(), stamps.end());

    std::vector<PoseDetectionData> all_detection_data;
    all_detection_data.reserve(data.size());

    for (const auto &stamp: unique_timestamps) {
        for (const auto &data_at_stamp: data[std::to_string(stamp)]) {
            std::string cam_name = data_at_stamp["rostopic"];
            Eigen::Matrix2Xd corners2d(2, data_at_stamp["corners2d"].size());
            for (int i = 0; i < corners2d.cols(); i++) {
                const auto &point = data_at_stamp["corners2d"][i];
                corners2d(0, i) = point[0];
                corners2d(1, i) = point[1];
            }
            Eigen::Matrix3Xd model_points(3, data_at_stamp["model_points"].size());
            for (int i = 0; i < model_points.cols(); i++) {
                const auto &point = data_at_stamp["model_points"][i];
                model_points(0, i) = point[0];
                model_points(1, i) = point[1];
                model_points(2, i) = point[2];
            }

            Eigen::Matrix4d T_cam_board = Eigen::Matrix4d::Identity();
            const auto T_data = data_at_stamp["T_camera_board"];
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    T_cam_board(i, j) = T_data[i][j];
                }
            }
            all_detection_data.push_back({cam_name, stamp, T_cam_board});
        }
    }

    std::cout << all_detection_data.size() << std::endl;
    std::cout << all_detection_data[0] << std::endl;
    std::sort(all_detection_data.begin(), all_detection_data.end());
    std::cout << "is sorted " << std::is_sorted(all_detection_data.begin(), all_detection_data.end()) << std::endl;
    unsigned long time_tolerance_ns = time_tolerance_secs * 1e9;
    std::cout << "Time tolerance nanoseconds " << time_tolerance_ns << std::endl;
    std::vector<std::pair<unsigned long, unsigned long>> correspondence_pairs;

    std::map<std::string, unsigned long> camera_name_to_node_descriptor;
    // Create a graph object
    Graph pose_detection_graph;

    for (size_t i = 0; i < all_detection_data.size(); i++) {
        const PoseDetectionData &data_i = all_detection_data[i];
        if (camera_name_to_node_descriptor.count(data_i.name) == 0) {
            camera_name_to_node_descriptor[data_i.name] = add_vertex(pose_detection_graph);
            put(vertex_name, pose_detection_graph, camera_name_to_node_descriptor[data_i.name], data_i.name);
        }

        for (size_t j = i + 1; j < all_detection_data.size(); j++) {
            const PoseDetectionData &data_j = all_detection_data[j];
            if (data_j.timestamp - data_i.timestamp > time_tolerance_ns) {
                break;
            }
            if (data_j.name == data_i.name) {
                continue;
            }
            correspondence_pairs.push_back({i, j});
            if (camera_name_to_node_descriptor.count(data_j.name) == 0) {
                camera_name_to_node_descriptor[data_j.name] = add_vertex(pose_detection_graph);
                put(vertex_name, pose_detection_graph, camera_name_to_node_descriptor[data_j.name], data_j.name);
            }

            add_edge_if_not_exists(camera_name_to_node_descriptor[data_i.name],
                                   camera_name_to_node_descriptor[data_j.name], pose_detection_graph);

        }
    }
    std::cout << "Found " << correspondence_pairs.size() << " pairs" << std::endl;
    // Check connectivity
    std::vector<int> component(num_vertices(pose_detection_graph));
    int num = connected_components(pose_detection_graph, &component[0]);

    std::cout << "Number of connected components: " << num << " out of " << component.size() << " nodes." << std::endl;
    if (num == 1) {
        std::cout << "The graph is connected." << std::endl;
    } else {
        std::cout << "The graph is not connected." << std::endl;
    }

    // Write the graph to a Graphviz dot file
    std::ofstream file("graph.dot");
    write_graphviz(file, pose_detection_graph, make_label_writer(get(vertex_name, pose_detection_graph)));

    return 0;
}
