//
// Created by liuyh on 26/4/2023.
//

#ifndef MAPF_DP_DISCRETEONLINESIMULATOR_H
#define MAPF_DP_DISCRETEONLINESIMULATOR_H

#include "../dependency_graph/NodeDependencyGraph.h"
#include "DiscreteSimulator.h"
#include "OnlineSimulator.h"


class DiscreteOnlineSimulator : public DiscreteSimulator, public OnlineSimulator {
protected:
    NodeDependencyGraph depGraph;

//    std::vector<std::vector<std::pair<size_t, unsigned int>>> deadEndStates;
//    std::unordered_map<unsigned int, std::vector<SharedNodePair>> sharedNodes;

//    std::vector<std::vector<unsigned int>> pathTopoNodeIds;
//    Graph::topo_graph_t topoGraph;

public:
    DiscreteOnlineSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed), depGraph(graph, agents, paths, firstAgentArrivingTimestep) {}

    unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep,
                          unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;

    bool onlineOpt = false;
    bool groupDetermined = false;

private:
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

    std::pair<size_t, size_t> feasibilityCheck();
};


#endif//MAPF_DP_DISCRETEONLINESIMULATOR_H
