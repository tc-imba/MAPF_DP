//
// Created by liuyh on 5/3/2024.
//

#ifndef MAPF_DP_DISCRETEBTPGSIMULATOR_H
#define MAPF_DP_DISCRETEBTPGSIMULATOR_H

#include "../../BTPG/inc/Sim.hpp"
#include "BTPGSimulator.h"
#include "DiscreteSimulator.h"


class DiscreteBTPGSimulator : public DiscreteSimulator, public BTPGSimulator {
public:
    int algorithmIdx;
    int timeInterval;

    DiscreteBTPGSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed, int algorithmIdx = 1,
                          int timeInterval = 60000)
        : Simulator(graph, agents, seed), DiscreteSimulator(graph, agents, seed), algorithmIdx(algorithmIdx), timeInterval(timeInterval) {}

    unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep, unsigned int delayStart = INT_MAX,
                          unsigned int delayInterval = INT_MAX) override;


private:
    std::shared_ptr<btpg::BTPG> btpg;
    std::shared_ptr<btpg::Sim> sim;

    void initBTPGVariables();

    void decideMovableAgents(std::vector<int> &movableAgents, unsigned int currentTimestep, unsigned int delayStart, unsigned int delayInterval);
};


#endif//MAPF_DP_DISCRETEBTPGSIMULATOR_H
