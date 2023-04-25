//
// Created by liuyh on 25/4/2023.
//

#ifndef MAPF_DP_DEFAULTSIMULATOR_H
#define MAPF_DP_DEFAULTSIMULATOR_H

#include "Simulator.h"

class DefaultSimulator : virtual public Simulator {
public:
    void print(std::ostream &out) const override;

    void printState(size_t i, unsigned int state) override;
};


#endif //MAPF_DP_DEFAULTSIMULATOR_H
