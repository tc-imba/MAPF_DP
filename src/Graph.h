//
// Created by liu on 28/4/2021.
//

#ifndef MAPF_DP_GRAPH_H
#define MAPF_DP_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

struct Agent {
    unsigned int start, goal, current;
    unsigned int state;
    unsigned int timestep;
    unsigned int waitingTimestep;
};

class Graph {
private:
    std::vector<std::vector<double>> distances;
    std::vector<std::vector<unsigned int>> gridGraphIds;
    unsigned int nodeNum;
    unsigned int edgeNum;
    double minDP = 0;
    double maxDP = 0.5;

    void generateGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename, size_t seed);

    void generateUnweightedGraph(std::vector<std::vector<char>> &gridGraph);

    bool isConnected();

    void saveGridGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename);

    bool loadGridGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename);

public:
    struct Node {
        char type;
    };

    struct Edge {
        unsigned int length;
        double dp;
        double _distance;
    };

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Node, Edge> graph_t;

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS > topo_graph_t;
    typedef boost::graph_traits<topo_graph_t>::vertex_descriptor topo_vertex_t;

    graph_t g;

    Graph();

    void initDelayProbability(double minDP, double maxDP);

    void generateDelayProbability(size_t seed, double minDP, double maxDP);

    void generateRandomGraph(unsigned int height, unsigned int width, unsigned int obstacles,
                             const std::string &filename = "", size_t seed = 0);

    void generateWareHouse(unsigned int deliveryWidth, unsigned int maxX, unsigned int maxY,
                           const std::string &filename = "", size_t seed = 0);

    void generateHardCodedGraph(const std::string &filename = "", size_t seed = 0);

    void calculateAllPairShortestPath(const std::string &filename = "", bool dp = false);

    void calculateUnweightedAllPairShortestPath();

    double getHeuristic(unsigned int nodeId1, unsigned int nodeId2);

    const Edge &getEdge(unsigned int nodeId1, unsigned int nodeId2);

    [[nodiscard]] auto getNodeNum() const { return nodeNum; }

    [[nodiscard]] auto getEdgeNum() const { return edgeNum; }

    std::vector<Agent> generateRandomAgents(unsigned int agentNum, size_t seed = 0);

    std::vector<Agent> generateWarehouseAgents(unsigned int agentNum, size_t seed = 0, bool swap = false);

    std::vector<Agent> generateHardCodedAgents(unsigned int agentNum);
};


#endif //MAPF_DP_GRAPH_H
