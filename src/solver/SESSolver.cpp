//
// Created by liuyh on 9/12/2024.
//

#include "SESSolver.h"
bool SESSolver::solve() {
    auto node = std::make_shared<CBSNode>();
    node->constraint = {INVALID, INVALID, INVALID};
    node->plans.resize(agents.size());
    node->parent = nullptr;
    node->depth = 0;

    /*const auto &paths = std::get<1>(simulator->adg);

    for (size_t i = 0; i < agents.size(); i++) {
        node->plans[i] = std::make_shared<AgentPlan>();
        const auto &path = paths[i];
        for (const auto &[location, value]: path) {
            auto nodeId = graph.getNodeIdByGridPos(location.first, location.second);
            if (nodeId == std::numeric_limits<unsigned int>::max()) return false;
            node->plans[i]->add(nodeId);
        }
    }
*/

    auto _simulator = std::make_unique<ses::Simulator>(simulator->adg);

    int agentCnt = ses::get_agentCnt(_simulator->adg);
    std::vector<int> movable(agentCnt, 0);
    _simulator->checkMovable(movable);
    auto &paths = get<1>(_simulator->adg);

    /*for (int agent = 0; agent < agentCnt; agent++) {
        if (movable[agent] == 1) {
            auto &start = paths[agent][_simulator->states[agent]];
            auto &end = paths[agent][_simulator->states[agent] + 1];
            std::cerr << "agent " << agent << ": (" << start.first.first << ","
                      << start.first.second << ")->(" << end.first.first << ","
                      << end.first.second << "), state: " << _simulator->states[agent]
                      << std::endl;
        } else {
            auto &start = paths[agent][_simulator->states[agent]];
            std::cerr << "agent " << agent << ": (" << start.first.first << ","
                      << start.first.second
                      << ") stop, state: " << _simulator->states[agent] << std::endl;
        }
    }*/

    int stepSpend = _simulator->step(false);

    if (stepSpend <= 0) { return false; }

    std::vector<std::deque<ses::Location>> expanded_paths;

    for (int agent = 0; agent < agentCnt; agent++) {
        std::deque<ses::Location> expanded_path;
        expanded_path.push_back(get<0>((paths[agent])[0]));
        expanded_paths.emplace_back(std::move(expanded_path));
    }

    while (stepSpend != 0) {
        for (int agent = 0; agent < agentCnt; agent++) {
            auto new_location = get<0>((paths[agent])[(_simulator->states[agent])]);
            if (!((ses::same_locations(new_location, (expanded_paths[agent]).back())) &&
                  ((size_t) (_simulator->states[agent]) ==
                   (paths[agent]).size() - 1))) {
                (expanded_paths[agent]).push_back(new_location);
            }
        }
        stepSpend = _simulator->step(false);
    }

    for (int agent = 0; agent < agentCnt; agent++) {
        node->plans[agent] = std::make_shared<AgentPlan>();
        auto &expanded_path = expanded_paths[agent];
        for (auto &location: expanded_path) {
            auto nodeId = graph.getNodeIdByGridPos(location.first, location.second);
            if (nodeId == std::numeric_limits<unsigned int>::max()) return false;
            node->plans[agent]->add(nodeId);
        }
    }

    solution = node;
    return true;
}
