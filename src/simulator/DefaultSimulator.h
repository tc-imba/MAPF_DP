//
// Created by liuyh on 25/4/2023.
//

#ifndef MAPF_DP_DEFAULTSIMULATOR_H
#define MAPF_DP_DEFAULTSIMULATOR_H

#include "Simulator.h"

class DefaultSimulator : virtual public Simulator {
public:
    bool replanMode = false;
    bool prioritizedReplan = false;
    bool prioritizedOpt = false;
    size_t partialReplanCount = 0;
    double partialReplanTime = 0;
    size_t fullReplanCount = 0;
    double fullReplanTime = 0;


    void print(std::ostream &out) const override;

    void printState(std::ostream &os, size_t i, unsigned int state) override;

    double replan();
};


#endif //MAPF_DP_DEFAULTSIMULATOR_H
