//
// Created by liu on 3/1/2022.
//

#ifndef MAPF_DP_CONTINUOUSDEFAULTSIMULATOR_H
#define MAPF_DP_CONTINUOUSDEFAULTSIMULATOR_H

#include "ContinuousSimulator.h"
#include "DefaultSimulator.h"

class ContinuousDefaultSimulator : public ContinuousSimulator, public DefaultSimulator {
public:
    ContinuousDefaultSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed) {}

    int simulate(double &currentTimestep, unsigned int maxTimeStep,
                 unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;
};

#endif //MAPF_DP_CONTINUOUSDEFAULTSIMULATOR_H
