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

    bool isHeuristicFeasibilityCheck = true;
    bool isHeuristicCycleCheck = true;
    bool isOnlyCycleCheck = false;

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
    std::set<size_t> blocked, unblocked, moved, ready, unshared;

    void initSharedNodes(size_t i, size_t j);

    void updateSharedNode(unsigned int nodeId, size_t agentId, unsigned int state);

    void initSimulation();

    void initChecks();

    void unsharedCheck();

    void neighborCheck();

    void deadEndCheck();

    void naiveCycleCheckHelper(std::vector<size_t> &readyList, size_t length, size_t start, size_t current,
                               std::vector<bool> &check, std::vector<size_t> &maxReadyList);

    void naiveCycleCheck();

    void heuristicCycleCheck();

    void cycleCheck();

    std::pair<size_t, size_t> feasibilityCheckHelper(
            std::list<SharedNodePair> &sharedNodesList
//            std::vector<std::pair<unsigned int, unsigned int>> &addedEdges
    );

    std::pair<size_t, size_t> feasibilityCheck();

};


#endif //MAPF_DP_ONLINESIMULATOR_H
