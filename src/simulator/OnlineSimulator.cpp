//
// Created by liuyh on 25/4/2023.
//

#include "OnlineSimulator.h"

void OnlineSimulator::writeSimulationOutput() {
    Simulator::writeSimulationOutput();
    resultJson["cycle_check_count"] = cycleCheckCount;
    resultJson["cycle_check_agents"] = cycleCheckAgents;
    resultJson["unblocked_agents"] = unblockedAgents;
//    resultJson["feasibilityCheckCount"] = feasibilityCheckCount;
//    resultJson["feasibilityCheckUnsettledEdgePairsCount"] = feasibilityCheckUnsettledCount;
//    resultJson["feasibilityCheckLoopCount"] = feasibilityCheckLoopCount;
//    resultJson["feasibilityCheckTopoCount"] = feasibilityCheckTopoCount;
//    resultJson["feasibilityCheckRecursionCount"] = feasibilityCheckRecursionCount;
}


void OnlineSimulator::print(std::ostream &out) const {
    out << executionTime << ","
        << firstAgentArrivingExecutionTime << ","
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


void OnlineSimulator::printState(std::ostream &os, size_t i, unsigned int state) {
    auto currentNodeId = paths[i][state / 2];
    os << "(" << state << "," << currentNodeId;
    if (state % 2 == 1) {
        auto nextNodeId = paths[i][state / 2 + 1];
        os << "-" << nextNodeId;
    }
    os << ")";
}


void OnlineSimulator::printSets(const std::string &title, double currentTimestep) {
    if (debug) {
        std::ostringstream oss;
        auto size = ready.size() + moved.size() + blocked.size() + unblocked.size() + unshared.size();
        oss << title << " timestep: " << currentTimestep << " agents: " << size << " ";
        oss << "ready: ";
        for (auto j: ready) {
            oss << j << " ";
        }
        oss << "moved: ";
        for (auto j: moved) {
            oss << j << " ";
        }
        oss << "blocked: ";
        for (auto j: blocked) {
            oss << j << " ";
        }
        oss << "unblocked: ";
        for (auto j: unblocked) {
            oss << j << " ";
        }
        oss << "unshared: ";
        for (auto j: unshared) {
            oss << j << " ";
        }
        SPDLOG_DEBUG("{}", oss.str());
    }
}
