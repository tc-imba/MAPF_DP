//
// Created by 51439 on 2/17/2025.
//

#ifndef CONSTRAINTTABLE_H
#define CONSTRAINTTABLE_H

#include <boost/icl/interval_set.hpp>

#include "Graph.h"
#include "solver/Solver.h"

class ConstraintTable {
public:

protected:
    std::shared_ptr<Graph> graph;

    // node -> time range, or edge -> time range
    // transform node and edge index to uniform index
    // index  0  to   |V|-1   -> node index 0 to |V|-1
    // index |V| to |V|+|E|-1 -> edge index 0 to |E|-1
    std::unordered_map<size_t, boost::icl::interval_set<double> > constraints;
    double maxConstraintTimestep = 0;

public:
    void addConstraint(size_t nodeId, double startTimestep, double endTimestep);

    void addConstraint(size_t nodeId1, size_t nodeId2, double startTimestep, double endTimestep);

    void addConstraints(const AgentPlan &plan);
};


#endif //CONSTRAINTTABLE_H
