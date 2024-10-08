//
// Created by fu on 01/10/24.
//

#include <gtboxcalibration/detectiongraphutils.h>

namespace gt_box::graph_utils {


    bool PoseDetectionData::operator<(const PoseDetectionData &rhs) const {
        return timestamp < rhs.timestamp;
    }

    std::ostream &operator<<(std::ostream &os, const PoseDetectionData &data) {
        os << "name: " << data.name << " timestamp: " << data.timestamp << " T_camera_board: "
           << data.T_camera_board;
        return os;
    }

    bool PoseDetectionData::operator>(const PoseDetectionData &rhs) const {
        return rhs < *this;
    }

    bool PoseDetectionData::operator<=(const PoseDetectionData &rhs) const {
        return !(rhs < *this);
    }

    bool PoseDetectionData::operator>=(const PoseDetectionData &rhs) const {
        return !(*this < rhs);
    }

    void add_edge_if_not_exists(Graph::vertex_descriptor u, Graph::vertex_descriptor v, Graph &g) {
        if (!edge(u, v, g).second) {
            boost::add_edge(u, v, g);
        }
    }

    Edge addEdge(Graph::vertex_descriptor u, Graph::vertex_descriptor v, Graph &g) {
        const auto existing_edge = boost::edge(u,v,g);
        if (!existing_edge.second) {
            auto new_edge = boost::add_edge(u, v, g);
            boost::put(boost::edge_capacity, g, new_edge.first, 1);
            return new_edge.first;
        } else {
            boost::put(boost::edge_capacity, g, existing_edge.first,
                       boost::get(boost::edge_capacity, g, existing_edge.first) + 1);
            return existing_edge.first;
        }
    }
}