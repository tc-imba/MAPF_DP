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
    size_t replanCount = 0;
    size_t replanBeforeArrivingCount = 0;
    size_t fullReplanCount = 0;
    size_t fullReplanBeforeArrivingCount = 0;


    void print(std::ostream &out) const override;

    void printState(size_t i, unsigned int state) override;

    double replan();
};


#endif //MAPF_DP_DEFAULTSIMULATOR_H
