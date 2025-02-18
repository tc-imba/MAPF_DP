//
// Created by 51439 on 2/17/2025.
//

#include "ConstraintTable.h"

void ConstraintTable::addConstraint(size_t nodeId, double startTimestep, double endTimestep) {
    auto &intervalSet = constraints[nodeId];
    auto interval = boost::icl::interval<double>::right_open(startTimestep, endTimestep);
    intervalSet.add(interval);
    if (endTimestep < std::numeric_limits<double>::infinity() && endTimestep > maxConstraintTimestep) {
        maxConstraintTimestep = endTimestep;
    } else if (endTimestep == std::numeric_limits<double>::infinity() && startTimestep > maxConstraintTimestep) {
        maxConstraintTimestep = startTimestep;
    }
}

void ConstraintTable::addConstraint(size_t nodeId1, size_t nodeId2, double startTimestep, double endTimestep) {
    auto &edge = graph->getEdge(nodeId1, nodeId2);
    addConstraint(edge.index + graph->getNodeNum(), startTimestep, endTimestep);
}

void ConstraintTable::addConstraints(const AgentPlan &plan) {
    for (size_t i = 0; i < plan.path.size() - 1; i++) {
        const auto &label = plan.path[i];
        const auto &nextLabel = plan.path[i + 1];
        addConstraint(label.nodeId, label.estimatedTime, nextLabel.estimatedTime);
        addConstraint(label.nodeId, nextLabel.nodeId, label.estimatedTime, nextLabel.estimatedTime);
    }
    // add infinite waiting
    const auto &lastLabel = plan.path.back();
    addConstraint(lastLabel.nodeId, lastLabel.estimatedTime, std::numeric_limits<double>::infinity());
}
