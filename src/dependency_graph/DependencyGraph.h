//
// Created by liuyh on 7/8/2023.
//

#ifndef MAPF_DP_DEPENDENCYGRAPH_H
#define MAPF_DP_DEPENDENCYGRAPH_H

#include "../Graph.h"
#include "../solver/Solver.h"


class DependencyGraph {
protected:
    struct SharedNodePair {
        size_t agentId1;
        unsigned int state1;
        size_t agentId2;
        unsigned int state2;
    };

    Graph &graph;
    std::vector<Agent> &agents;
    std::vector<std::vector<unsigned int>> &paths;
    const double &firstAgentArrivingTimestep;


public:
    bool debug = false;
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
};


#endif//MAPF_DP_DEPENDENCYGRAPH_H
