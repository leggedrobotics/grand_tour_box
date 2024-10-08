#include <Eigen/Dense>
#include <string>
#include <boost/graph/adjacency_list.hpp>

namespace gt_box::graph_utils {
    using namespace boost;
    // Define a simple undirected graph type with properties for vertex names
    typedef property<edge_capacity_t, unsigned int> EdgeCapacityProperty;
    typedef property<vertex_name_t, std::string> NamedVertexProperty;
    typedef adjacency_list<vecS, vecS, undirectedS, NamedVertexProperty, EdgeCapacityProperty> Graph;
    typedef graph_traits<Graph>::edge_descriptor Edge;

    struct PoseDetectionData {
        std::string name;
        unsigned long long timestamp;
        Eigen::Matrix4d T_camera_board;

        friend std::ostream &operator<<(std::ostream &os, const PoseDetectionData &data);

        bool operator<(const PoseDetectionData &rhs) const;

        bool operator>(const PoseDetectionData &rhs) const;

        bool operator<=(const PoseDetectionData &rhs) const;

        bool operator>=(const PoseDetectionData &rhs) const;
    };

    // Function to add edges only if they do not exist
    void add_edge_if_not_exists(Graph::vertex_descriptor u, Graph::vertex_descriptor v,
                                Graph &g);

    Edge addEdge(Graph::vertex_descriptor u, Graph::vertex_descriptor v,
            Graph &g);

}
