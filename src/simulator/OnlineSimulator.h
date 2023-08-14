//
// Created by liuyh on 25/4/2023.
//

#ifndef MAPF_DP_ONLINESIMULATOR_H
#define MAPF_DP_ONLINESIMULATOR_H

#include "Simulator.h"

class OnlineSimulator : virtual public Simulator {
public:
    bool isHeuristicFeasibilityCheck = true;
    bool isHeuristicCycleCheck = true;
    bool isOnlyCycleCheck = false;
    bool isFeasibilityType = false;

    size_t feasibilityCheckCount = 0;
    size_t feasibilityCheckTypes[4] = {0, 0, 0, 0};
    size_t cycleCheckCount = 0;
    size_t cycleCheckAgents = 0;
    size_t unblockedAgents = 0;
//    size_t feasibilityCheckIteration[2] = {0, 0};
//    size_t feasibilityCheckIterationTemp = 0;

    size_t feasibilityCheckUnsettledCount = 0;
    size_t feasibilityCheckTopoCount = 0;
    size_t feasibilityCheckLoopCount = 0;
    size_t feasibilityCheckRecursionCount = 0;

    void print(std::ostream &out) const override;

    void printState(std::ostream &os, size_t i, unsigned int state) override;

    void printSets(const std::string &title);

protected:
    struct SharedNodePair {
        size_t agentId1;
        unsigned int state1;
        size_t agentId2;
        unsigned int state2;
    };

    std::vector<std::vector<unsigned int>> paths;
    std::unordered_map<size_t, size_t> nodeAgentMap;
    std::set<size_t> blocked, unblocked, moved, ready, unshared, keepMoving;
};


#endif //MAPF_DP_ONLINESIMULATOR_H
