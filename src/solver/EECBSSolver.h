//
// Created by liu on 18/12/2022.
//

#ifndef MAPF_DP_EECBSSOLVER_H
#define MAPF_DP_EECBSSOLVER_H

#include "Solver.h"

class EECBSSolver : public Solver {
private:
    bool prioritizedReplan = false;
    std::vector<Agent> savedAgents;
    std::vector<std::shared_ptr<AgentPlan >> savedPlans;
    std::vector<size_t> plannedAgentsMap;

public:
    EECBSSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType) : Solver(graph, agents, makeSpanType) {};

    std::string getSolverName() override { return "eecbs"; }

    bool readSolution(std::ifstream &fin);

    bool solve() override;

    bool solveWithPrioritizedReplan() override;

    void saveObstacles(const std::string &filename);
};

#endif //MAPF_DP_EECBSSOLVER_H
