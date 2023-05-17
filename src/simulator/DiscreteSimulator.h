//
// Created by liuyh on 25/4/2023.
//

#ifndef MAPF_DP_DISCRETESIMULATOR_H
#define MAPF_DP_DISCRETESIMULATOR_H

#include "Simulator.h"

class DiscreteSimulator : virtual public Simulator {
protected:
    std::set<unsigned int> delayedSet;
public:
    void updateDelayedSet(unsigned int currentTimestep, unsigned int delayStart, unsigned int delayInterval);
};


#endif //MAPF_DP_DISCRETESIMULATOR_H
