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


void OnlineSimulator::printSets(const std::string &title) {
    if (debug) {
        auto size = ready.size() + moved.size() + blocked.size() + unblocked.size() + unshared.size();
        std::cout << title << " " << size << " ";
        std::cout << "ready: ";
        for (auto j: ready) {
            std::cout << j << " ";
        }
        std::cout << "moved: ";
        for (auto j: moved) {
            std::cout << j << " ";
        }
        std::cout << "blocked: ";
        for (auto j: blocked) {
            std::cout << j << " ";
        }
        std::cout << "unblocked: ";
        for (auto j: unblocked) {
            std::cout << j << " ";
        }
        std::cout << "unshared: ";
        for (auto j: unshared) {
            std::cout << j << " ";
        }
        std::cout << std::endl;
    }
}
