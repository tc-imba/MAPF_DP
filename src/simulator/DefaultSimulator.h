//
// Created by liu on 3/1/2022.
//

#ifndef MAPF_DP_DEFAULTSIMULATOR_H
#define MAPF_DP_DEFAULTSIMULATOR_H

#include "Simulator.h"

class DefaultSimulator : public Simulator {
public:
    size_t firstAgentArrivingTimestep = 0;
    double executionTime = 0;


    DefaultSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed) {}

    int simulate(double &currentTimestep, unsigned int maxTimeStep,
                 unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;

    void print(std::ostream &out) const override;
};

#endif //MAPF_DP_DEFAULTSIMULATOR_H
