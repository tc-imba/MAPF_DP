//
// Created by liu on 3/1/2022.
//

#ifndef MAPF_DP_DEFAULTSIMULATOR_H
#define MAPF_DP_DEFAULTSIMULATOR_H

#include "Simulator.h"

class DefaultSimulator : public Simulator {
public:
    DefaultSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed) {}

    int simulate(unsigned int &currentTimestep, unsigned int maxTimeStep, unsigned int pauseTimestep = 0) override;

    void print(std::ostream &out) const override;
};

#endif //MAPF_DP_DEFAULTSIMULATOR_H
