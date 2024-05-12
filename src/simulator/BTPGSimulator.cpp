//
// Created by liuyh on 5/3/2024.
//

#include "BTPGSimulator.h"

void BTPGSimulator::print(std::ostream &out) const {
    out << executionTime << ","
        << firstAgentArrivingExecutionTime << ","
        << firstAgentArrivingTimestep;
}

void BTPGSimulator::printState(std::ostream &os, size_t i, unsigned int state) {

}