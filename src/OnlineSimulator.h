//
// Created by liu on 7/1/2022.
//

#ifndef MAPF_DP_ONLINESIMULATOR_H
#define MAPF_DP_ONLINESIMULATOR_H

#include "Simulator.h"

class OnlineSimulator : public Simulator {
public:
    OnlineSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed) {}

    bool simulate(unsigned int &currentTimestep, unsigned int maxTimeStep) override;

private:
    std::vector<std::vector<unsigned int>> paths;
    std::vector<std::vector<std::pair<size_t, unsigned int> > > deadEndStates;

    std::unordered_map<size_t, size_t> nodeAgentMap;
    std::set<size_t> blocked, unblocked, moved, ready;

    void initSharedNodes(size_t i, size_t j);

    void initSimulation();

    void initChecks();

    void neighborCheck();

    void deadEndCheck();

};


#endif //MAPF_DP_ONLINESIMULATOR_H
