//
// Created by liuyh on 26/4/2023.
//

#ifndef MAPF_DP_DISCRETEDEFAULTSIMULATOR_H
#define MAPF_DP_DISCRETEDEFAULTSIMULATOR_H

#include "DiscreteSimulator.h"
#include "DefaultSimulator.h"

class DiscreteDefaultSimulator : public DiscreteSimulator, public DefaultSimulator {
public:
    std::vector<std::vector<unsigned int> > agentRealPaths;

    DiscreteDefaultSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) :
            Simulator(graph, agents, seed) {}

    unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep,
                 unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;
};


#endif //MAPF_DP_DISCRETEDEFAULTSIMULATOR_H
