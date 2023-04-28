//
// Created by liuyh on 26/4/2023.
//

#ifndef MAPF_DP_DISCRETEONLINESIMULATOR_H
#define MAPF_DP_DISCRETEONLINESIMULATOR_H

#include "DiscreteSimulator.h"
#include "OnlineSimulator.h"


class DiscreteOnlineSimulator : public DiscreteSimulator, public OnlineSimulator {
public:
    DiscreteOnlineSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) :
            Simulator(graph, agents, seed) {}

    int simulate(double &currentTimestep, unsigned int maxTimeStep,
                 unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;

private:
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

    bool isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2);

    std::pair<size_t, size_t> feasibilityCheckHelper(
            std::list<SharedNodePair> &sharedNodesList,
            bool recursive
//            std::vector<std::pair<unsigned int, unsigned int>> &addedEdges
    );

    std::pair<size_t, size_t> feasibilityCheckTest(bool recursive);

    std::pair<size_t, size_t> feasibilityCheck();

};


#endif //MAPF_DP_DISCRETEONLINESIMULATOR_H
