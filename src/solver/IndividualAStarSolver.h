//
// Created by liuyh on 20/3/2023.
//

#ifndef MAPF_DP_INDIVIDUALASTARSOLVER_H
#define MAPF_DP_INDIVIDUALASTARSOLVER_H

#include "Solver.h"

class IndividualAStarSolver : public Solver {
public:
    IndividualAStarSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType)
            : Solver(graph, agents, makeSpanType) {}

    std::string getSolverName() override { return "individual"; }

    void init() override;

    bool solve() override;

    std::shared_ptr<AgentPlan> search(unsigned int agentId);
};


#endif //MAPF_DP_INDIVIDUALASTARSOLVER_H
