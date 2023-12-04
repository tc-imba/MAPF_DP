//
// Created by liuyh on 7/8/2023.
//

#ifndef MAPF_DP_DEPENDENCYGRAPH_H
#define MAPF_DP_DEPENDENCYGRAPH_H

#include "../Graph.h"
#include "../solver/Solver.h"


class DependencyGraph {
public:
    struct SharedNodePair {
        size_t agentId1;
        unsigned int state1;
        size_t agentId2;
        unsigned int state2;
    };

    struct SDGNode {
        size_t agentId;
        unsigned int state;
    };

    friend bool operator<(const SDGNode &lhs, const SDGNode &rhs) {
        if (lhs.agentId == rhs.agentId) return lhs.state < rhs.state;
        return lhs.agentId < rhs.agentId;
    }

    friend bool operator==(const SDGNode &lhs, const SDGNode &rhs) {
        return lhs.agentId == rhs.agentId && lhs.state == rhs.state;
    }

    friend std::ostream &operator<<(std::ostream &out, const SDGNode &node) {
        return out << "(" << node.agentId << "," << node.state << ")";
    }

    struct SDGEdge {
        SDGNode source, dest;
    };

    friend bool operator<(const SDGEdge &lhs, const SDGEdge &rhs) {
        return lhs.source == rhs.source ? lhs.dest < rhs.dest : lhs.source < rhs.source;
    }

    friend bool operator==(const SDGEdge &lhs, const SDGEdge &rhs) {
        return lhs.source == rhs.source && lhs.dest == rhs.dest;
    }

    friend std::ostream &operator<<(std::ostream &out, const SDGEdge &edge) {
        return out << edge.source << "->" << edge.dest;
    }

    typedef std::pair<SDGNode, SDGNode> SDGNodePair;
    typedef std::pair<SDGEdge, SDGEdge> SDGEdgePair;

protected:

    Graph &graph;
    std::vector<Agent> &agents;
    std::vector<std::vector<unsigned int>> &paths;
    const double &firstAgentArrivingTimestep;
    std::vector<std::vector<double>> timestamps;


public:
    bool debug = false;
    bool onlineOpt = false;
    std::shared_ptr<Solver> solver = nullptr;
    std::vector<std::vector<unsigned int>> pathTopoNodeIds;
    Graph::topo_graph_t topoGraph;

    //    std::vector<std::vector<unsigned int>> paths;
    std::vector<std::vector<std::pair<size_t, unsigned int>>> deadEndStates;
    std::unordered_map<unsigned int, std::vector<SharedNodePair>> sharedNodes;

    size_t feasibilityCheckUnsettledCount = 0;
    size_t feasibilityCheckTopoCount = 0;
    size_t feasibilityCheckLoopCount = 0;
    size_t feasibilityCheckRecursionCount = 0;

    DependencyGraph(Graph &graph, std::vector<Agent> &agents, std::vector<std::vector<unsigned int>> &paths, const double &firstAgentArrivingTimestep) : graph(graph), agents(agents), paths(paths), firstAgentArrivingTimestep(firstAgentArrivingTimestep){};

    std::pair<unsigned int, unsigned int> getTopoEdgeBySDGEdge(const SDGEdge &edge) const {
        return std::make_pair(pathTopoNodeIds[edge.source.agentId][edge.source.state],
                              pathTopoNodeIds[edge.dest.agentId][edge.dest.state]);
    };

    bool isEdgeInTopoGraph(unsigned int nodeId1, unsigned int nodeId2);

    bool isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2);
};


#endif//MAPF_DP_DEPENDENCYGRAPH_H
