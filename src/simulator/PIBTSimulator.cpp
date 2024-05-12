//
// Created by liuyh on 29/4/2023.
//

#include "PIBTSimulator.h"


void PIBTSimulator::print(std::ostream &out) const {
    out << executionTime << ","
        << firstAgentArrivingExecutionTime << ","
        << firstAgentArrivingTimestep;
}

void PIBTSimulator::printState(std::ostream &os, size_t i, unsigned int state) {

}
