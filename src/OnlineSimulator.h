//
// Created by liu on 7/1/2022.
//

#ifndef MAPF_DP_ONLINESIMULATOR_H
#define MAPF_DP_ONLINESIMULATOR_H

#include "Simulator.h"

class OnlineSimulator : public Simulator {
public:
    OnlineSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed) {}

    int simulate(unsigned int &currentTimestep, unsigned int maxTimeStep, unsigned int pauseTimestep = 0) override;

private:
    struct SharedNodePair {
        size_t agentId1;
        unsigned int state1;
        size_t agentId2;
        unsigned int state2;
    };

    std::vector<std::vector<unsigned int>> paths;
    std::vector<std::vector<std::pair<size_t, unsigned int> > > deadEndStates;
    std::unordered_map<unsigned int, std::vector<SharedNodePair>> sharedNodes;

    std::vector<std::vector<unsigned int>> pathTopoNodeIds;
    Graph::topo_graph_t topoGraph;

    std::unordered_map<size_t, size_t> nodeAgentMap;
    std::set<size_t> blocked, unblocked, moved, ready;

    void initSharedNodes(size_t i, size_t j);

    void initSimulation();

    void initChecks();

    void neighborCheck();

    void deadEndCheck();

    void cycleCheck();

    void feasibilityCheck();

};


#endif //MAPF_DP_ONLINESIMULATOR_H
