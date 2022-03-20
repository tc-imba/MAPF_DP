//
// Created by liu on 3/1/2022.
//

#include "DefaultSimulator.h"
#include <iostream>
#include <random>

int DefaultSimulator::simulate(unsigned int &currentTimestep, const unsigned int maxTimeStep, unsigned int pauseTimestep) {
//    if (!success) return true;
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

    for (const auto &node: nodes) {
        nodeStates[node.first] = 0;
        if (debug) {        /*
            std::cout << node.first << " ";
            for (const auto &item : node.second) {
                std::cout << "(" << item.first << "," << item.second << ")->";
            }
            std::cout << std::endl;
        */
        }
    }
    if (debug) {
        for (unsigned int i = 0; i < agents.size(); i++) {
//        if (agents[i].current == agents[i].goal) continue;
            std::cout << "agent " << i << "(" << agents[i].start << "->" << agents[i].goal << "): ";
            for (const auto &label: solution->plans[i]->path) {
                std::cout << "(" << label.state << "," << label.nodeId << ")->";
            }
            std::cout << std::endl;
//        nodeStates[agents[i].start] = 0;
        }
    }


//    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> distribution(0, 1);

    for (; currentTimestep < maxTimeStep; currentTimestep++) {
        if (debug) {
            std::cout << "begin timestep " << currentTimestep << std::endl;
        }
        int count = 0;
        std::vector<unsigned int> unblocked;

//        if (pauseTimestep > 0 && currentTimestep == pauseTimestep) {
//            updateDelayedSet(currentTimestep);
//        }
        updateDelayedSet(currentTimestep);

        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            if (state + 1 >= solution->plans[i]->path.size()) {
                if (debug) {
                    std::cout << "agent " << i << ": completed" << std::endl;
                }
                ++count;
            } else if (delayedSet.find(i) != delayedSet.end()) {
                if (debug) {
                    std::cout << "agent " << i << ": delayed" << std::endl;
                }
                agents[i].timestep = currentTimestep;
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
                    unblocked.emplace_back(i);
                }
            }
        }
        for (auto i: unblocked) {
            auto &state = agents[i].state;
            const auto &label = solution->plans[i]->path[state];
            const auto &nextLabel = solution->plans[i]->path[state + 1];
            const auto &nextNode = nodes[nextLabel.nodeId];
            if (label.nodeId != nextLabel.nodeId) {
                auto &edge = graph.getEdge(label.nodeId, nextLabel.nodeId);
                auto waitingTimestep = 1;
//                auto waitingTimestep = (unsigned int) floor(10.0 * (edge.dp - 0.5) + 2);
//                auto waitingTimestep = (unsigned int) floor(1 / (1.0 - edge.dp));
//                auto waitingTimestep = (unsigned int) floor(exp(5.0 * (edge.dp - 0.5)) + 1);
                if (++agents[i].waitingTimestep < waitingTimestep) {
//                std::mt19937 generator(
//                        combineRandomSeed(label.nodeId, nextLabel.nodeId, currentTimestep, seed));
//                double rand = distribution(generator);
//                if (rand < edge.dp) {
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
            agents[i].waitingTimestep = 0;
            ++state;
            ++nodeStates[label.nodeId];
        }


//        std::cout << count << std::endl;
        if (count >= agents.size()) {
            break;
        }
    }

//    std::cout << "window " << window << ": " << averageMakeSpan() << std::endl;

/*    bool unfinish = false;
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (agents[i].current != agents[i].goal) {
            unfinish = true;
//            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << ", goal " << agents[i].goal << std::endl;
            break;
        }
    }*/
    return countCompletedAgents();
}
