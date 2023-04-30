//
// Created by liuyh on 29/4/2023.
//

#ifndef MAPF_DP_PIBTSIMULATOR_H
#define MAPF_DP_PIBTSIMULATOR_H

#include "Simulator.h"

class PIBTSimulator : virtual public Simulator {
public:
    void print(std::ostream &out) const override;

    void printState(size_t i, unsigned int state) override;
};


#endif //MAPF_DP_PIBTSIMULATOR_H
