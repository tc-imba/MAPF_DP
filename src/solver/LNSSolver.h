//
// Created by liuyh on 3/1/2025.
//

#ifndef MAPF_DP_LNSSOLVER_H
#define MAPF_DP_LNSSOLVER_H

#include "Solver.h"


class LNSSolver : public Solver {
protected:
    std::string solverBinaryFile;

public:
    LNSSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType,
              std::string solverBinaryFile)
        : Solver(graph, agents, makeSpanType),
          solverBinaryFile(std::move(solverBinaryFile)) {};

    std::string getSolverName() override { return "lns"; }

    bool readSolution(std::ifstream &fin);

    bool solve() override;
};


#endif//MAPF_DP_LNSSOLVER_H
