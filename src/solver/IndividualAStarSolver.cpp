//
// Created by liuyh on 20/3/2023.
//

#include "IndividualAStarSolver.h"
#include <queue>

void IndividualAStarSolver::init() {
    Solver::init();
    success = false;
}

bool IndividualAStarSolver::solve() {
//    std::cout << agents.size() << std::endl;

    auto node = std::make_shared<CBSNode>();
    node->constraint = {INVALID, INVALID, INVALID};
    node->plans.resize(agents.size());
    node->parent = nullptr;
    node->depth = 0;

    for (size_t i = 0; i < agents.size(); i++) {
//        std::cout << "agent: " << i << std::endl;
        node->plans[i] = search(i);
        if (node->plans[i]->path.empty()) return false;
    }

    solution = node;
    success = true;
    return true;
}

std::shared_ptr<AgentPlan> IndividualAStarSolver::search(unsigned int agentId) {
//    std::cout << "agent " << agentId << std::endl;

    std::shared_ptr<AgentPlan> plan = std::make_shared<AgentPlan>();
    AStarNodePtrComp comp;
    std::priority_queue<AStarNodePtr, std::vector<AStarNodePtr>, AStarNodePtrComp> open(comp);
    std::unordered_map<size_t, AStarNodePtr> aStarNodes;

    auto &agent = agents[agentId];
    auto node = std::make_shared<AStarNode>();
    node->label = {agent.current, 0, 0, graph.getHeuristic(agent.current, agent.goal)};
    node->parent = nullptr;
    aStarNodes.emplace(node->label.nodeId, node);
    open.emplace(std::move(node));

    unsigned int count = 0;

    while (!open.empty()) {
        auto current = open.top();
        open.pop();
        if (current->visited) {
            continue;
        }
        count++;
        current->visited = true;

//        std::cout << current->label.nodeId << " " << current->label.state << " "
//                  << current->label.estimatedTime << " " << current->label.heuristic << " "
//                  << current->label.estimatedTime + current->label.heuristic
//                  << std::endl;

        if (current->label.nodeId == agent.goal) {
//            std::cout << "end" << std::endl;
            AStarNode *temp = current.get();
            while (temp) {
//                std::cout << temp->label.nodeId << std::endl;
                plan->path.emplace_front(temp->label);
                temp = temp->parent.get();
            }
            return plan;
        }

        auto neighborEdges = boost::out_edges(current->label.nodeId, graph.g);

        for (auto edge: boost::make_iterator_range(neighborEdges)) {
            unsigned int neighborNodeId = edge.m_target;

            if (current->parent && current->parent->label.nodeId == neighborNodeId) continue;

            Label label = {neighborNodeId, current->label.state + 1, (double) current->label.state + 1, 0};
            label.heuristic = graph.getHeuristic(label.nodeId, agent.goal);

            auto it = aStarNodes.find(label.nodeId);
            auto newNode = std::make_shared<AStarNode>();
            newNode->label = label;
            newNode->parent = current;

            if (it != aStarNodes.end()) {
                auto oldNode = it->second;
                if (!oldNode->visited && comp(oldNode, newNode)) {
                    oldNode.swap(newNode);
                    open.emplace(std::move(oldNode));
//                    std::cout << "update " << label.nodeId << " " << label.state << " " << label.heuristic << std::endl;
                }
            } else {
                aStarNodes.emplace_hint(it, newNode->label.nodeId, newNode);
                open.emplace(std::move(newNode));
//                std::cout << "add " << label.nodeId << " " << label.state << " " << label.heuristic << std::endl;
            }
        }
    }

    assert(plan->path.empty());
    return plan;
}