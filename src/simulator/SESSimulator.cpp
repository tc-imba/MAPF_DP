//
// Created by liuyh on 27/11/2024.
//

#include "SESSimulator.h"


void SESSimulator::print(std::ostream &out) const {
    out << executionTime << ","
        << firstAgentArrivingExecutionTime << ","
        << firstAgentArrivingTimestep;
}

void SESSimulator::printState(std::ostream &os, size_t i, unsigned int state) {

}
