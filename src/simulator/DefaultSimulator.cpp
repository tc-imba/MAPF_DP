//
// Created by liuyh on 25/4/2023.
//

#include "DefaultSimulator.h"

void DefaultSimulator::print(std::ostream &out) const {
    out << executionTime << ","
        << firstAgentArrivingTimestep;
}

void DefaultSimulator::printState(size_t i, unsigned int state) {
    auto currentNodeId = solver->solution->plans[i]->path[state].nodeId;
    std::cout << "(" << state << "," << currentNodeId << ")";
}
