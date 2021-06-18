//
// Created by liu on 27/4/2021.
//

#include "CBSSolver.h"
#include <iostream>
#include <random>

std::shared_ptr<CBSSolver::AgentPlan> CBSSolver::focalSearch(CBSNode &cbsNode, unsigned int agentId, double key) {
    std::shared_ptr<AgentPlan> plan = std::make_shared<AgentPlan>();
    FocalNodePtrComp comp(key + 1 - currentTimestep);
    std::priority_queue<FocalNodePtr, std::vector<FocalNodePtr>, FocalNodePtrComp> open(comp);
    std::unordered_map<Label, FocalNodePtr, LabelHash, LabelEqual> focalNodes;
//    auto compareLabel = LabelComp{};

    auto &agent = agents[agentId];
    auto node = std::make_shared<FocalNode>();
    node->label = {agent.current, 0, 0, graph.getHeuristic(agent.current, agent.goal)};
    node->conflicts = 0;
    node->parent = nullptr;
    focalNodes.emplace(node->label, node);
    open.emplace(std::move(node));

    unsigned int count = 0;

    while (!open.empty()) {
        auto current = open.top();
        open.pop();
        if (current->visited) {
            continue;
        }
        count++;
//        if (count >= 1000000) break;
        current->visited = true;
//        std::cout << current->label.nodeId << " " << current->label.state << " "
//                  << current->label.estimatedTime << " " << current->label.heuristic << " "
//                  << current->label.estimatedTime + current->label.heuristic << ", "
//                  << "conflicts: " << current->conflicts
//                  << std::endl;
        if (current->label.nodeId == agent.goal) {
//            if (current->label.state <= lastStates[agentId]) {
//                std::cout << "error" << std::endl;
//            }
            // reconstruct path
            bool reconstruct = true;
            if (constraintSet.find(current->label) != constraintSet.end()) {
                reconstruct = false;
            }

            if (reconstruct) {
                FocalNode *temp = current.get();
                while (temp) {
                    plan->path.emplace_front(temp->label);
                    temp = temp->parent.get();
                }
                if (debug) {
                    std::cout << "agent: " << agentId << ", estimate: "
                              << currentTimestep - 1 + current->label.estimatedTime
                              << ", conflicts: " << current->conflicts << ", steps: " << count << std::endl;
                }
                return plan;
            }
        }

        std::vector<Label> newLabels;
        auto neighborEdges = boost::out_edges(current->label.nodeId, graph.g);
        bool needWaiting = true;

        for (auto edge : boost::make_iterator_range(neighborEdges)) {
            unsigned int neighborNodeId = edge.m_target;
            Label newLabel = {neighborNodeId, current->label.state + 1, 0, 0};
            if (constraintSet.find(newLabel) == constraintSet.end()) {
                newLabel.estimatedTime = getEstimate(newLabel, current->label)
                                         + graph.g[edge].length / (1.0 - graph.g[edge].dp);
                newLabel.heuristic = graph.getHeuristic(newLabel.nodeId, agent.goal);
                if (newLabel.heuristic < current->label.heuristic) {
                    needWaiting = false;
                }
                newLabels.push_back(newLabel);
            }
        }
//        if (newLabels.empty()) {
        if (needWaiting) {
            Label newLabel = {current->label.nodeId, current->label.state + 1, 0, 0};
            if (constraintSet.find(newLabel) == constraintSet.end()) {
                newLabel.estimatedTime = getEstimate(newLabel, current->label) + 1;
                newLabel.heuristic = graph.getHeuristic(newLabel.nodeId, agent.goal);
                newLabels.push_back(newLabel);
            }
        }
        for (auto &label : newLabels) {
            auto result = findConflict(label, agentId, true, false);
            unsigned int conflict = 0;
            if (!result.empty()) conflict = 1;
            /*auto cit = conflictMap.find(label);
            if (cit != conflictMap.end()) {
                if (cit->second.size() > 1) {
                    conflict = 1;
                } else if (cit->second.size() == 1 && cit->second.front() != agentId) {
                    conflict = 1;
                }
            }*/

            auto it = focalNodes.find(label);
            auto newNode = std::make_shared<FocalNode>();
//            label.heuristic = graph.getHeuristic(label.nodeId, agent.goal);
            newNode->label = label;
            newNode->parent = current;
            newNode->conflicts = current->conflicts + conflict;

            if (it != focalNodes.end()) {
                auto oldNode = it->second;
                assert(oldNode->label.state == label.state && oldNode->label.nodeId == label.nodeId);
//                label.heuristic = oldNode->label.heuristic;
                if (!oldNode->visited && comp(oldNode, newNode)) {
                    oldNode.swap(newNode);
//                    oldNode->label = label;
//                    oldNode->parent = current;
//                    oldNode->conflicts = current->conflicts + conflict;
                    open.emplace(std::move(oldNode));
                }
            } else {
//                auto newNode = std::make_shared<FocalNode>();
//                label.heuristic = graph.getHeuristic(label.nodeId, agent.goal);
//                newNode->label = label;
//                newNode->parent = current;
//                newNode->conflicts = current->conflicts + conflict;
                focalNodes.emplace_hint(it, newNode->label, newNode);
                open.emplace(std::move(newNode));
            }
        }
    }

    assert(plan->path.empty());
    return plan;
}


std::vector<CBSSolver::Constraint>
CBSSolver::findConflict(const Label &label, unsigned int agentId, bool find, bool edit) {
    std::vector<CBSSolver::Constraint> result;
    auto it1 = conflictMap.find(label);
    if (find) {
        if (it1 != conflictMap.end() && it1->second.front() != agentId) {
            // type 1 conflict
            result.push_back({it1->second.front(), label.nodeId, label.state});
            result.push_back({agentId, label.nodeId, label.state});
//            return result;
        } else {
            auto temp = label;
            temp.state = label.state + 1;
            auto it2 = conflictMap.find(temp);
            if (it2 != conflictMap.end() && it2->second.front() != agentId) {
                // type 2 conflict
                result.push_back({it2->second.front(), label.nodeId, label.state + 1});
                result.push_back({agentId, label.nodeId, label.state});
//                return result;
            } else if (label.state > 0) {
                temp.state = label.state - 1;
                it2 = conflictMap.find(temp);
                if (it2 != conflictMap.end() && it2->second.front() != agentId) {
                    // type 2 conflict
                    result.push_back({agentId, label.nodeId, label.state});
                    result.push_back({it2->second.front(), label.nodeId, label.state - 1});
//                    return result;
                }
            }
        }
    }
    if (edit) {
        if (it1 == conflictMap.end()) {
            it1 = conflictMap.emplace_hint(it1, label, std::vector<unsigned int>());
        }
        it1->second.emplace_back(agentId);
    }
    return result;
}


std::vector<CBSSolver::Constraint> CBSSolver::findConflicts(CBSNode &cbsNode) {
    std::vector<CBSSolver::Constraint> result;
    std::vector<std::pair<unsigned int, unsigned int>> lastStates(agents.size(), {0, 0});
    for (unsigned agentId = 0; agentId < cbsNode.plans.size(); agentId++) {
        lastStates[agentId].second = agentId;
    }

    estimateMap.clear();
    conflictMap.clear();


    for (unsigned int state = 0;; state++) {
        unsigned int count = 0;
        for (unsigned agentId = 0; agentId < agents.size(); agentId++) {
            const auto &plan = cbsNode.plans[agentId];
            if (!plan || state >= plan->path.size()) {
                ++count;
                continue;
            }
            const auto &label = plan->path[state];

            auto ittt = goalAgentMap.find(label.nodeId);
            if (ittt != goalAgentMap.end() && ittt->second != agentId) {
                if (lastStates[ittt->second].first < label.state) {
                    lastStates[ittt->second].first = label.state;
                    lastStates[ittt->second].second = agentId;
                }
//                lastStates[ittt->second] = std::max(lastStates[ittt->second], label.state);
            }
            if (!allConstraint && label.state >= window) continue;

            // update estimate map
            auto it = estimateMap.find(label.nodeId);
            if (it == estimateMap.end()) {
                it = estimateMap.emplace_hint(it, label.nodeId, std::map<unsigned int, double>());
            }
            auto itt = it->second.find(label.state);
            if (itt == it->second.end()) {
                it->second.emplace_hint(itt, label.state, label.estimatedTime);
            } else if (itt->second < label.estimatedTime) {
                itt->second = label.estimatedTime;
            }

//            auto it1 = conflictMap.find(label);
            auto temp = findConflict(label, agentId, result.empty(), true);
            if (!temp.empty()) {
                result.swap(temp);
            }

        }
        if (count >= agents.size()) break;
    }

/*    for (unsigned agentId = 0; agentId < cbsNode.plans.size(); agentId++) {
        const auto &plan = cbsNode.plans[agentId];
        for (const auto &label : plan.path) {

*//*            if (result.empty()) {
                if (it1 != conflictMap.end() && it1->second.front() != agentId) {
                    // type 1 conflict
                    result.push_back({it1->second.front(), label.nodeId, label.state});
                    result.push_back({agentId, label.nodeId, label.state});
                    continue;
//                return result;
                }
                auto temp = label;
                temp.state = label.state + 1;
                auto it2 = conflictMap.find(temp);
                if (it2 != conflictMap.end() && it2->second.front() != agentId) {
                    // type 2 conflict
                    result.push_back({it2->second.front(), label.nodeId, label.state + 1});
                    result.push_back({agentId, label.nodeId, label.state});
                    continue;
//                return result;
                }
                if (label.state > 0) {
                    temp.state = label.state - 1;
                    it2 = conflictMap.find(temp);
                    if (it2 != conflictMap.end() && it2->second.front() != agentId) {
                        // type 2 conflict
                        result.push_back({agentId, label.nodeId, label.state});
                        result.push_back({it2->second.front(), label.nodeId, label.state - 1});
                        continue;
//                    return result;
                    }
                }
            }

            if (it1 == conflictMap.end()) {
                it1 = conflictMap.emplace_hint(it1, label, std::vector<unsigned int>());
            }
            it1->second.emplace_back(agentId);*//*
        }
    }*/

    for (auto &[nodeId, m] : estimateMap) {
        double max = 0;
        for (auto &[state, estimateTime] : m) {
            max = estimateTime = std::max(max, estimateTime);
        }
    }

    if (result.empty()) {
        for (unsigned agentId = 0; agentId < cbsNode.plans.size(); agentId++) {
            const auto &plan = cbsNode.plans[agentId];
            if (plan && !plan->path.empty()) {
                const auto &label = plan->path.back();
                if (label.state <= lastStates[agentId].first && lastStates[agentId].second != agentId) {
                    result.push_back({lastStates[agentId].second, label.nodeId, lastStates[agentId].first});
//                    if (label.state == 0) {
                    result.push_back({agentId, label.nodeId, label.state});
//                    }
//                    std::cout << "last state constraint: " << label.nodeId << " "
//                              << agentId << " " << label.state << " "
//                              << lastStates[agentId].second << " " << lastStates[agentId].first << " "
//                              << std::endl;
                    break;
                }
            }
        }
    }

//    assert(result.empty());
    return result;
}

double CBSSolver::approxAverageMakeSpan(CBSNode &cbsNode) {
    double result = 0;
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (makeSpanType == MakeSpanType::MAXIMUM) {
            if (agents[i].current == agents[i].goal && cbsNode.plans[i]->path.size() == 1) {
                result = std::max(result, (double) agents[i].timestep);
            } else if (!cbsNode.plans[i]->path.empty()) {
                result = std::max(result, currentTimestep - 1 + cbsNode.plans[i]->path.back().estimatedTime);
            } else {
                result = std::max(result, 1e9);
            }
        } else if (makeSpanType == MakeSpanType::AVERAGE) {
            if (agents[i].current == agents[i].goal && cbsNode.plans[i]->path.size() == 1) {
                result += agents[i].timestep / (double) agents.size();
            } else if (!cbsNode.plans[i]->path.empty()) {
                result += (currentTimestep - 1 + cbsNode.plans[i]->path.back().estimatedTime) / (double) agents.size();
            } else {
                result = std::max(result, 1e9);
            }
        }
    }
    return result;
}


double CBSSolver::getEstimate(const Label &label, const Label &parentLabel) {
    double result = parentLabel.estimatedTime;
    auto it = estimateMap.find(label.nodeId);
    if (it != estimateMap.end() && !it->second.empty()) {
        auto it2 = it->second.lower_bound(label.state);
        if (it2 != it->second.begin()) {
            --it2;
            result = std::max(result, it2->second);
        }
    }
    return result;
}


void CBSSolver::init(unsigned int _seed, MakeSpanType makeSpanType) {
    Solver::init(makeSpanType);
    seed = _seed;
    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].current = agents[i].start;
        agents[i].timestep = 0;
    }
    currentTimestep = 1;
}

void CBSSolver::initCBS(unsigned int _window) {
    success = false;
    window = _window;

    goalAgentMap.clear();
    for (unsigned int i = 0; i < agents.size(); i++) {
        goalAgentMap[agents[i].goal] = i;
    }

    std::priority_queue<CBSNodePtr, std::vector<CBSNodePtr>, CBSNodePtrComp> newQueue;
    queue.swap(newQueue);
    estimateMap.clear();
    constraintSet.clear();

    auto node = std::make_shared<CBSNode>();
    node->constraint = {INVALID, INVALID, INVALID};
    node->plans.resize(agents.size());
    node->parent = nullptr;
    node->depth = 0;
    generateConstraintSet(*node);

    for (unsigned int i = 0; i < agents.size(); i++) {
        findConflicts(*node);
        node->plans[i] = focalSearch(*node, i, currentTimestep - 1);
//        std::cout << i << " " << node->plans[i].path.back().estimatedTime << std::endl;
        if (node->plans[i]->path.empty()) {
            return;
        }
    }
    node->key = approxAverageMakeSpan(*node);
    if (window >= std::numeric_limits<unsigned int>::max() / 2) {
        std::cout << "lower bound: " << node->key << std::endl;
    }
    queue.emplace(std::move(node));
}

bool CBSSolver::step() {
    if (success) return false;
    if (queue.empty()) return false;
    auto node = queue.top();
    queue.pop();

    if (debug) {
        std::cout << "estimate: " << node->key << ", depth: " << node->depth << " | "
                  << node->constraint.nodeId << " " << node->constraint.state << " " << node->constraint.agentId
                  << std::endl;
    }

    auto conflict = findConflicts(*node);

    if (conflict.empty()) {
        success = true;
        solution = node;
//        std::cout << "no conflict" << std::endl;
        return false;
    }
//    std::cout


    for (auto &constraint : conflict) {
        if (constraint.state == 0) continue;
        auto newNode = std::make_shared<CBSNode>();
        newNode->constraint = constraint;
        newNode->plans = node->plans;
        newNode->parent = node;
        auto constraintNum = generateConstraintSet(*newNode);
//        findConflict(*node);
        double searchKey;
        if (makeSpanType == MakeSpanType::MAXIMUM) {
            searchKey = node->key;
        } else if (makeSpanType == MakeSpanType::AVERAGE) {
            searchKey = currentTimestep - 1 + node->plans[constraint.agentId]->path.back().estimatedTime;
        }
//        searchKey = node->key;

        auto newPlan = focalSearch(*newNode, constraint.agentId, searchKey);
        if (!newPlan->path.empty()) {
            newNode->plans[constraint.agentId] = newPlan;
            newNode->key = approxAverageMakeSpan(*newNode);
            newNode->depth = node->depth + 1;
//            auto temp = findConflict(*newNode);
//            if (!temp.empty() && temp.front() == conflict.front() && temp.back() == conflict.back()) {
//                std::cout << "error" << std::endl;
//            }
            if (newPlan->path.size() > window) {
                bool alwaysStay = true;
                for (unsigned int i = 1; i < window; i++) {
                    if (newPlan->path[i].nodeId != agents[constraint.agentId].current) {
                        alwaysStay = false;
                        break;
                    }
                }
                if (alwaysStay) {
                    newNode->key += 1e5;
                }
            }

            if (debug) {
                std::cout << "add constraint: " << newNode->key << ", depth: " << newNode->depth << " | "
                          << constraint.nodeId << " " << constraint.state << " " << constraint.agentId << std::endl;
            }

            queue.emplace(std::move(newNode));
        } else {
            if (debug) {
                std::cout << "search failed" << std::endl;
            }
        }
    }

    // save memory
    node->plans.clear();
    return true;
}

unsigned int CBSSolver::generateConstraintSet(CBSSolver::CBSNode &cbsNode) {
    constraintSet.clear();
    auto agentId = cbsNode.constraint.agentId;
    if (agentId >= agents.size()) return 0;
    CBSNode *temp = &cbsNode;
    unsigned int result = 0;
    while (temp) {
        if (temp->constraint.agentId == agentId) {
            Label label = {temp->constraint.nodeId, temp->constraint.state, 0, 0};
            constraintSet.emplace(label);
        }
        temp = temp->parent.get();
        ++result;
    }
    return result;
}

bool CBSSolver::simulate() {
    if (!success) return true;
    std::unordered_map<unsigned int, std::vector<std::pair<unsigned int, unsigned int>>> nodes;
//    std::vector<unsigned int> agentStates(agents.size(), 0);
    std::unordered_map<unsigned int, int> nodeStates;

    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].state = 0;
    }

    for (unsigned int state = 0;; state++) {
        int count = 0;
        for (unsigned int i = 0; i < agents.size(); i++) {
            if (state >= solution->plans[i]->path.size()) {
                ++count;
            } else {
                const auto &label = solution->plans[i]->path[state];
                nodes[label.nodeId].emplace_back(label.state, i);
            }
        }
        if (count >= agents.size()) {
            break;
        }

    }
//    std::cout << nodes.size() << std::endl;

    for (const auto &node : nodes) {
        nodeStates[node.first] = 0;
        /*if (debug) {
            std::cout << node.first << " ";
            for (const auto &item : node.second) {
                std::cout << "(" << item.first << "," << item.second << ")->";
            }
            std::cout << std::endl;
        }*/
/*        */
    }
    if (debug) {
        for (unsigned int i = 0; i < agents.size(); i++) {
//        if (agents[i].current == agents[i].goal) continue;
            std::cout << "agent " << i << ": ";
            for (const auto &label : solution->plans[i]->path) {
                std::cout << "(" << label.state << "," << label.nodeId << ")->";
            }
            std::cout << std::endl;
//        nodeStates[agents[i].start] = 0;
        }
    }


//    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> distribution(0, 1);

    unsigned int maxTimeStep = currentTimestep + window;

    for (; currentTimestep < maxTimeStep; currentTimestep++) {
        if (debug) {
            std::cout << "begin timestep " << currentTimestep << std::endl;
        }
        int count = 0;
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            if (state + 1 >= solution->plans[i]->path.size()) {
//                std::cout << "agent " << i << ": completed" << std::endl;
                ++count;
            } else {
                agents[i].timestep = currentTimestep;
                const auto &label = solution->plans[i]->path[state];
                const auto &nextLabel = solution->plans[i]->path[state + 1];
                const auto &nextNode = nodes[nextLabel.nodeId];
                auto nextNodeState = nodeStates[nextLabel.nodeId];
//                std::cout << label.nodeId << " " << nextLabel.nodeId << " " << nextNode.size() << std::endl;
                if (label.nodeId == nextLabel.nodeId) {
                    nextNodeState++;
                }
                assert(nextNodeState < nextNode.size());
                auto next = nextNode[nextNodeState];
                if (next.first == nextLabel.state && next.second == i) {
                    // try to move
                    if (label.nodeId != nextLabel.nodeId) {
                        auto &edge = graph.getEdge(label.nodeId, nextLabel.nodeId);
                        std::mt19937 generator(
                                combineRandomSeed(label.nodeId, nextLabel.nodeId, currentTimestep, seed));
                        double rand = distribution(generator);
                        if (rand < edge.dp) {
                            if (debug) {
                                std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") delay"
                                          << std::endl;
                            }
                            continue;
                        }
                    }
                    if (debug) {
                        std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ")->("
                                  << state + 1 << "," << nextLabel.nodeId << ")" << std::endl;
                    }
                    agents[i].current = nextLabel.nodeId;
                    ++state;
                    ++nodeStates[label.nodeId];
                }
            }
        }
//        std::cout << count << std::endl;
        if (count >= agents.size()) {
            break;
        }
    }

//    std::cout << "window " << window << ": " << averageMakeSpan() << std::endl;

    bool unfinish = false;
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (agents[i].current != agents[i].goal) {
            unfinish = true;
//            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << ", goal " << agents[i].goal << std::endl;
            break;
        }
    }
    return unfinish;
}
