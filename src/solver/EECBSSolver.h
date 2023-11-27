//
// Created by liu on 18/12/2022.
//

#ifndef MAPF_DP_EECBSSOLVER_H
#define MAPF_DP_EECBSSOLVER_H

#include "Solver.h"

#include <utility>

class EECBSSolver : public Solver {
protected:
    std::string solverBinaryFile;

public:
    double suboptimality;

    EECBSSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType, std::string solverBinaryFile, double suboptimality) :
        Solver(graph, agents, makeSpanType), solverBinaryFile(std::move(solverBinaryFile)), suboptimality(suboptimality) {};

    std::string getSolverName() override { return "eecbs"; }

    bool readSolution(std::ifstream &fin);

    bool solve() override;

    void saveObstacles(const std::string &filename);
};

#endif //MAPF_DP_EECBSSOLVER_H
