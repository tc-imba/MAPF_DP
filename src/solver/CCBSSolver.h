//
// Created by liuyh on 10/4/2023.
//

#ifndef MAPF_DP_CCBSSOLVER_H
#define MAPF_DP_CCBSSOLVER_H

#include "Solver.h"

class CCBSSolver : public Solver {
protected:
    std::string solverBinaryFile;
    std::string mapType;

public:
    CCBSSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType, std::string solverBinaryFile, std::string mapType) :
        Solver(graph, agents, makeSpanType), solverBinaryFile(std::move(solverBinaryFile)), mapType(std::move(mapType)) {};

    std::string getSolverName() override { return "ccbs"; }

    bool readSolution(const std::string &filename);

    bool solve() override;

    void saveConfig(const std::string &filename);
};


#endif //MAPF_DP_CCBSSOLVER_H
