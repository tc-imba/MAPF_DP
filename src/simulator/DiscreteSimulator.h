//
// Created by liuyh on 25/4/2023.
//

#ifndef MAPF_DP_DISCRETESIMULATOR_H
#define MAPF_DP_DISCRETESIMULATOR_H

#include "../delay/DiscreteFixedDelay.h"
#include "Simulator.h"

class DiscreteSimulator : virtual public Simulator {
protected:
    DiscreteFixedDelay discreteFixedDelay;
    //    std::set<unsigned int> delayedSet;
public:
    //    void updateDelayedSet(unsigned int currentTimestep, unsigned int delayStart, unsigned int delayInterval);

    DiscreteSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed)
        : Simulator(graph, agents, seed),
          discreteFixedDelay(agents.size(), seed, delayRatio) {};
};


#endif//MAPF_DP_DISCRETESIMULATOR_H
