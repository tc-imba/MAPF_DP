//
// Created by liuyh on 27/11/2024.
//

#ifndef MAPF_DP_DISCRETESESSIMULATOR_H
#define MAPF_DP_DISCRETESESSIMULATOR_H

#include "../../Switchable-Edge-Search/src/Algorithm/Astar.h"
#include "../../Switchable-Edge-Search/src/Algorithm/simulator.h"
#include "DiscreteSimulator.h"
#include "SESSimulator.h"


class DiscreteSESSimulator : public DiscreteSimulator, public SESSimulator {
protected:
    std::shared_ptr<ses::Simulator> _simulator;

public:
    DiscreteSESSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed)
        : Simulator(graph, agents, seed), DiscreteSimulator(graph, agents, seed) {}

    unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep, unsigned int delayStart = INT_MAX,
                          unsigned int delayInterval = INT_MAX) override;

    int step_wdelay(bool *delay_mark, std::vector<int> &delayed_agents);

    int step(const std::string &adgFileName);
};



#endif//MAPF_DP_DISCRETESESSIMULATOR_H
