#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/algorithm/string.hpp>
#include <cmath>

struct Node {
    size_t index;
    std::string coords;
    double x, y;
};

struct Edge {
    unsigned int index;
    double distance;
    double weight;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Node, Edge> graph_t;

int main() {
    std::string filename = "roadmap";
    std::ifstream graphMLFileIn(filename + ".xml");
//    std::cerr << "load DOT graph from " << filename << ".xml" << std::endl;

    graph_t g;

    boost::dynamic_properties dp;
    dp.property("coords", boost::get(&Node::coords, g));
    dp.property("weight", boost::get(&Edge::weight, g));

    boost::read_graphml(graphMLFileIn, g, dp);

    auto nodes = boost::vertices(g);
    for (auto it = nodes.first; it != nodes.second; ++it) {
        g[*it].index = boost::get(boost::vertex_index, g, *it);
        std::vector<std::string> coordinates;
        boost::split(coordinates, g[*it].coords, boost::is_any_of(","));
        g[*it].x = std::strtod(coordinates[0].c_str(), nullptr);
        g[*it].y = std::strtod(coordinates[1].c_str(), nullptr);
//        std::cout << *it << " " << g[*it].index << " " << g[*it].x << " " << g[*it].y << std::endl;
    }

    for (auto it = nodes.first; it != nodes.second; ++it) {
        auto edges = boost::out_edges(g[*it].index, g);
        for (auto it2 = edges.first; it2 != edges.second; ++it2) {
            if (it2->m_source >= it2->m_target) continue;
            auto &src = g[it2->m_source];
            auto &dest = g[it2->m_target];
            auto dx = src.x - dest.x;
            auto dy = src.y - dest.y;
            g[*it2].distance = std::sqrt(dx * dx + dy * dy);
//            std::cout << it2->m_source << " " << it2->m_target << " " << g[*it2].distance << std::endl;
        }
    }

//    auto edges = boost::edges(g);
//    for (auto it = edges.first; it != edges.second; ++it) {
//
//    }
//

}