//
// Created by liuyh on 25/4/2023.
//

#include "OnlineSimulator.h"

void OnlineSimulator::print(std::ostream &out) const {
    out << executionTime << ","
        << firstAgentArrivingTimestep << ","
        << cycleCheckCount << ","
        << cycleCheckAgents << ","
        << unblockedAgents << ","
        << feasibilityCheckCount;
    for (int i = 0; i < 4; i++) {
        out << "," << feasibilityCheckTypes[i];
    }
    out << "," << feasibilityCheckUnsettledCount
        << "," << feasibilityCheckLoopCount
        << "," << feasibilityCheckTopoCount
        << "," << feasibilityCheckRecursionCount;
//    for (int i = 0; i < 2; i++) {
//        out << "," << feasibilityCheckIteration[i];
//    }
}


void OnlineSimulator::printState(size_t i, unsigned int state) {
    auto currentNodeId = paths[i][state / 2];
    std::cout << "(" << state << "," << currentNodeId;
    if (state % 2 == 1) {
        auto nextNodeId = paths[i][state / 2 + 1];
        std::cout << "-" << nextNodeId;
    }
    std::cout << ")";
}
