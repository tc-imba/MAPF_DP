//
// Created by liu on 28/4/2021.
//

#ifndef MAPF_DP_GRAPH_H
#define MAPF_DP_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <iostream>

struct Agent {
    unsigned int start, goal, current;
    unsigned int state;
    unsigned int timestep;
    unsigned int waitingTimestep;
    bool blocked;
    bool delayed;
};

class Graph {
private:
    std::vector<std::vector<double>> distances;
    std::vector<std::vector<unsigned int>> gridGraphIds;
    unsigned int nodeNum;
    unsigned int edgeNum;
    double minDP = 0;
    double maxDP = 0.5;
    size_t height, width;
    std::string graphFilename;

    void generateGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename, size_t seed,
                       bool write = true);

    void generateUnweightedGraph(std::vector<std::vector<char>> &gridGraph);

    bool isConnected();

    void saveGridGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename);

    bool loadGridGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename);

public:
    struct Node {
        size_t index;
        char type;
        unsigned int x;
        unsigned int y;
    };

    struct Edge {
        unsigned int index;
        double length;
        double dp;
        double _distance;
    };

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Node, Edge> graph_t;

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> topo_graph_t;
    typedef boost::graph_traits<topo_graph_t>::vertex_descriptor topo_vertex_t;

    graph_t g;
    bool debug = false;

    Graph();

    void initDelayProbability(double minDP, double maxDP);

    void generateDelayProbability(size_t seed, double minDP, double maxDP);

    void generateRandomGraph(unsigned int height, unsigned int width, unsigned int obstacles,
                             const std::string &filename = "", size_t seed = 0);

    void generateWareHouse(unsigned int deliveryWidth, unsigned int maxX, unsigned int maxY,
                           const std::string &filename = "", size_t seed = 0);

    void generateHardCodedGraph(const std::string &filename = "", size_t seed = 0);

    void generateFileGraph(const std::string &filename);

    void generateDOTGraph(const std::string &filename);

    void calculateAllPairShortestPath(const std::string &filename = "", bool dp = false);

    void calculateUnweightedAllPairShortestPath();

    double getHeuristic(unsigned int nodeId1, unsigned int nodeId2);

    const Node &getNode(unsigned int nodeId);

    const Edge &getEdge(unsigned int nodeId1, unsigned int nodeId2);

    [[nodiscard]] auto getNodeNum() const { return nodeNum; }

    [[nodiscard]] auto getEdgeNum() const { return edgeNum; }

    [[nodiscard]] auto getFileName() const { return graphFilename; }

    std::vector<Agent> generateRandomAgents(unsigned int agentNum, size_t seed = 0);

    std::vector<Agent> generateWarehouseAgents(unsigned int agentNum, size_t seed = 0, bool swap = false);

    std::vector<Agent> generateHardCodedAgents(unsigned int agentNum);

    void saveAgents(const std::string &mapName, const std::string &filename, const std::vector<Agent> &agents);

    unsigned int getNodeIdByGridPos(unsigned int x, unsigned int y);
};

class TopoGraphBFSVisitor : public boost::default_bfs_visitor {
public:
    struct done {
    };

    explicit TopoGraphBFSVisitor(const Graph::topo_graph_t::vertex_descriptor &v) : goal(v) {
    };

    void discover_vertex(const Graph::topo_graph_t::vertex_descriptor &v, const Graph::topo_graph_t &g) const {
        if (v == goal) throw done{};
    };

protected:
    const Graph::topo_graph_t::vertex_descriptor goal;
};


#endif //MAPF_DP_GRAPH_H
