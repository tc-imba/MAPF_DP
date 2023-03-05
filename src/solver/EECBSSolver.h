//
// Created by liu on 18/12/2022.
//

#ifndef MAPF_DP_EECBSSOLVER_H
#define MAPF_DP_EECBSSOLVER_H

#include "Solver.h"

#include <utility>

class EECBSSolver : public Solver {
private:
    bool prioritizedReplan = false;
    std::vector<Agent> savedAgents;
    std::vector<std::shared_ptr<AgentPlan >> savedPlans;
    std::vector<size_t> plannedAgentsMap;
    std::string solverBinaryFile;

public:
    EECBSSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType, std::string solverBinaryFile) :
        Solver(graph, agents, makeSpanType), solverBinaryFile(std::move(solverBinaryFile)) {};

    std::string getSolverName() override { return "eecbs"; }

    bool readSolution(std::ifstream &fin);

    bool solve() override;

    bool solveWithPrioritizedReplan() override;

    bool solveRetry();

    void saveObstacles(const std::string &filename);
};

#endif //MAPF_DP_EECBSSOLVER_H
