//
// Created by liu on 18/12/2022.
//

#ifndef MAPF_DP_EECBSSOLVER_H
#define MAPF_DP_EECBSSOLVER_H

#include "Solver.h"

class EECBSSolver : public Solver {
public:
    EECBSSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType) : Solver(graph, agents, makeSpanType) {};

    std::string getSolverName() override { return "eecbs"; }

    bool readSolution(std::ifstream &fin);

    bool solve() override;
};

#endif //MAPF_DP_EECBSSOLVER_H
