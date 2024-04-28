//
// Created by liuyh on 5/4/2024.
//

#ifndef MAPF_DP_CONTINUOUSPIBTSIMULATOR_H
#define MAPF_DP_CONTINUOUSPIBTSIMULATOR_H


#include "../solver/Solver.h"
#include "ContinuousSimulator.h"
#include "OnlineSimulator.h"

class ContinuousPIBTSimulator : public ContinuousSimulator, public OnlineSimulator {
public:
    ContinuousPIBTSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed)
        : Simulator(graph, agents, seed) {}

    unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep, unsigned int delayStart = INT_MAX,
                          unsigned int delayInterval = INT_MAX) override;

    std::vector<std::vector<double>> distances;
    std::vector<std::vector<Label>> plans;

private:
    bool PIBT(unsigned int agentId1, unsigned int agentId2);

    void calculateDistances();
};


#endif//MAPF_DP_CONTINUOUSPIBTSIMULATOR_H
