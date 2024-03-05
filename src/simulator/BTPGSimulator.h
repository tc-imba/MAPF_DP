//
// Created by liuyh on 5/3/2024.
//

#ifndef MAPF_DP_BTPGSIMULATOR_H
#define MAPF_DP_BTPGSIMULATOR_H

#include "Simulator.h"

class BTPGSimulator : virtual public Simulator {
public:
    void print(std::ostream &out) const override;

    void printState(std::ostream &os, size_t i, unsigned int state) override;
};


#endif//MAPF_DP_BTPGSIMULATOR_H
