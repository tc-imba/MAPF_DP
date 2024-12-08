//
// Created by liuyh on 27/11/2024.
//

#ifndef MAPF_DP_SESSIMULATOR_H
#define MAPF_DP_SESSIMULATOR_H

#include "Simulator.h"

class SESSimulator : virtual public Simulator {
public:
    void print(std::ostream &out) const override;

    void printState(std::ostream &os, size_t i, unsigned int state) override;
};

#endif//MAPF_DP_SESSIMULATOR_H
