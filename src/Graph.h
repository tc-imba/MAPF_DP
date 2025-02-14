//
// Created by liu on 28/4/2021.
//

#ifndef MAPF_DP_GRAPH_H
#define MAPF_DP_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <Mathematics/Vector2.h>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/core.h>
#include <spdlog/fmt/bundled/format.h>
#include <iostream>
#include <iomanip>

struct Agent {
    unsigned int start, goal, current;
    unsigned int state, newState;
    double timestep;
    double arrivingTimestep;
    bool blocked;
    bool delayed;
};

class Graph {
public:
    struct Node {
        size_t index;
        char type;
        double x;
        double y;
    };

    struct Edge {
        unsigned int index;
        double length;
        double dp;
        double _distance;
    };

    struct Neighbor {
        unsigned int x;
        unsigned int y;
        double length;
    };

    struct GraphMLNode {
        std::string coords;
        double x, y;
    };

    struct GraphMLEdge {
        double weight;
    };

    struct NodeEdgeState {
        bool isNode;
        double agentRadius;
        unsigned int srcNodeId;
        unsigned int destNodeId;
    };


private:
    std::vector<std::vector<double>> distances;
    std::vector<std::vector<unsigned int>> gridGraphIds;
    unsigned int nodeNum;
    unsigned int edgeNum;
    double minDP = 0;
    double maxDP = 0.5;
    size_t height, width;
    std::string graphFilename;
    unsigned int kNeighbor = 3;
    std::vector<gte::Vector2<int>> neighborDirections;
    double epsilon = 0.0000001;

    void generateGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename, size_t seed,
                       unsigned int kNeighbor = 2, bool write = true);

    void generateUnweightedGraph(std::vector<std::vector<char>> &gridGraph);

    bool isConnected();

    void saveGridGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename);

    bool loadGridGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename);

    void saveXMLGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename);

    void initKNeighbor(unsigned int kNeighbor);

    std::vector<Neighbor> getNeighbors(std::vector<std::vector<char>> &gridGraph, unsigned int x, unsigned int y);

    bool isPathConflictWithObstacle(std::vector<std::vector<char>> &gridGraph, gte::Vector2<int> start, gte::Vector2<int> goal, double distance);

public:
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Node, Edge> graph_t;
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, GraphMLNode, GraphMLEdge> graphml_graph_t;
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> topo_graph_t;
    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::directedS> agent_graph_t;
    typedef boost::graph_traits<topo_graph_t>::vertex_descriptor topo_vertex_t;

    graph_t g;
    bool debug = false;
    bool noCache = false;
    bool nodeNodeConflict = false;
    bool edgeEdgeConflict = false;
    bool nodeEdgeConflict = false;

    Graph();

    void initDelayProbability(double minDP, double maxDP);

    void generateDelayProbability(size_t seed, double minDP, double maxDP);

    void generateRandomGraph(unsigned int height, unsigned int width, unsigned int obstacles,
                             const std::string &filename = "", size_t seed = 0, unsigned int kNeighbor = 2);

    void generateWareHouse(unsigned int deliveryWidth, unsigned int maxX, unsigned int maxY,
                           const std::string &filename = "", size_t seed = 0);

    void generateHardCodedGraph(const std::string &filename = "", size_t seed = 0);

    void generateMAPFBenchmarkGraph(const std::string &filename, unsigned int kNeighbor);

    void generateDOTGraph(const std::string &filename);

    void generateGraphMLGraph(const std::string &filename);

    void calculateAllPairShortestPath(const std::string &filename = "", bool dp = false);

    void calculateUnweightedAllPairShortestPath();

    double getHeuristic(unsigned int nodeId1, unsigned int nodeId2);

    bool isConflict(const NodeEdgeState& state1, const NodeEdgeState& state2);

    const Node &getNode(unsigned int nodeId);

    const Edge *getEdgePtr(unsigned int nodeId1, unsigned int nodeId2);

    const Edge &getEdge(unsigned int nodeId1, unsigned int nodeId2);

    [[nodiscard]] auto getNodeNum() const { return nodeNum; }

    [[nodiscard]] auto getEdgeNum() const { return edgeNum; }

    [[nodiscard]] auto getKNeighbor() const { return kNeighbor; }

    [[nodiscard]] auto getFileName() const { return graphFilename; }

    std::vector<Agent> generateRandomAgents(unsigned int agentNum, size_t seed = 0);

    std::vector<Agent> generateWarehouseAgents(unsigned int agentNum, size_t seed = 0, bool swap = false);

    std::vector<Agent> generateHardCodedAgents(unsigned int agentNum);

    std::vector<Agent> loadXMLAgents(const std::string &filename, unsigned int agentNum, size_t skip = 0);

    std::vector<Agent> loadScenAgents(const std::string &filename, unsigned int agentNum, size_t skip = 0);

    void saveScenAgents(const std::string &mapName, const std::string &filename, const std::vector<Agent> &agents);

    void saveXMLAgents(const std::string &filename, const std::vector<Agent> &agents, const std::string &mapType);

    unsigned int getNodeIdByGridPos(unsigned int x, unsigned int y);

    unsigned int getNodeIdByRealPos(double x, double y);
};

class TopoGraphBFSVisitor : public boost::default_bfs_visitor {
public:
    struct done {
    };

    std::vector<unsigned int> path;

    explicit TopoGraphBFSVisitor(const Graph::topo_graph_t::vertex_descriptor &v) : goal(v) {
    };

    void discover_vertex(const Graph::topo_graph_t::vertex_descriptor &v, const Graph::topo_graph_t &g) {
//        path.emplace_back((unsigned int) v);
        if (v == goal) {
//            std::cout << "path: ";
//            for (auto &v : path) {
//                std::cout << v << " ";
//            }
//            std::cout << std::endl;
            throw done{};
        }
    };

protected:
    const Graph::topo_graph_t::vertex_descriptor goal;
};


#endif //MAPF_DP_GRAPH_H
