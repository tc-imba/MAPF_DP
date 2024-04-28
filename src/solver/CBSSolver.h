//
// Created by liu on 27/4/2021.
//

#ifndef MAPF_DP_CBSSOLVER_H
#define MAPF_DP_CBSSOLVER_H

#include "Solver.h"
#include <vector>
#include <queue>
#include <memory>
#include <limits>

#include <boost/functional/hash.hpp>

class CBSSolver : public Solver {
private:
    std::priority_queue<CBSNodePtr, std::vector<CBSNodePtr>, CBSNodePtrComp> queue;
    std::unordered_map<unsigned int, std::map<unsigned int, double> > estimateMap;
    std::unordered_map<unsigned int, unsigned int> goalAgentMap;
    std::unordered_set<Label, LabelHash, LabelEqual> constraintSet;
    std::unordered_map<Label, std::vector<unsigned int>, LabelHash, LabelEqual> conflictMap;
    unsigned int window = 1000;
//    unsigned int seed = 0;


    std::vector<Constraint> findConflict(const Label &label, unsigned int agentId, bool find, bool edit);

    std::vector<Constraint> findConflicts(CBSNode &cbsNode);

    double getEstimate(const Label &label, const Label &parentLabel);

    unsigned int generateConstraintSet(CBSNode &cbsNode);

public:
    CBSSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType, unsigned int window = 1000) :
        Solver(graph, agents, makeSpanType), window(window) {};

    std::string getSolverName() override { return "default"; }

    void init() override;

    bool step();

    bool solve() override;

    std::shared_ptr<AgentPlan> focalSearch(CBSNode &cbsNode, unsigned int agentId, double key);

//    void addPathIntoConstraints(std::shared_ptr<AgentPlan> plans);
};


#endif //MAPF_DP_CBSSOLVER_H
