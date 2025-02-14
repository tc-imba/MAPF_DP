//
// Created by liuyh on 26/4/2023.
//

#include "DiscreteDefaultSimulator.h"

unsigned int DiscreteDefaultSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                               unsigned int delayStart, unsigned int delayInterval) {
//    if (!success) return true;
    std::unordered_map<unsigned int, std::vector<std::pair<unsigned int, unsigned int>>> nodes;
//    std::vector<unsigned int> agentStates(agents.size(), 0);
    std::unordered_map<unsigned int, int> nodeStates;
    std::vector<unsigned int> savedStart(agents.size());
    std::vector<std::string> agentOutputStates(agents.size());
    agentRealPaths.clear();
    agentRealPaths.resize(agents.size());
    const auto &delayedSet = discreteFixedDelay.getDelayedSet();

    openOutputFiles();

    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].state = 0;
        agents[i].blocked = true;
        agents[i].delayed = false;
        savedStart[i] = agents[i].current = agents[i].start;
        agentRealPaths[i].emplace_back(agents[i].current);
    }

    bool refresh = true;

/*    for (const auto &node: nodes) {
        nodeStates[node.first] = 0;
        if (debug) {
            std::cout << node.first << " ";
            for (const auto &item : node.second) {
                std::cout << "(" << item.first << "," << item.second << ")->";
            }
            std::cout << std::endl;
        }
    }
    if (debug) {
        for (unsigned int i = 0; i < agents.size(); i++) {
//        if (agents[i].current == agents[i].goal) continue;
            std::cout << "agent " << i << "(" << agents[i].start << "->" << agents[i].goal << "): ";
            for (const auto &label: solver->solution->plans[i]->path) {
                std::cout << "(" << label.state << "," << label.nodeId << ")->";
            }
            std::cout << std::endl;
//        nodeStates[agents[i].start] = 0;
        }
    }*/


//    std::mt19937 generator(seed);
//    std::uniform_real_distribution<double> distribution(0, 1);

    if (outputFile.is_open()) {
        outputFile << 0 << std::endl;
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &node = graph.getNode(solver->solution->plans[i]->path[0].nodeId);
            outputFile << i << " " << node.index << " " << "node" << " " << "block" << std::endl;
        }
    }

    for (; currentTimestep + 1 < maxTimeStep; ) {
        executionTimeStart = std::chrono::steady_clock::now();
        if (refresh) {
            refresh = false;
            nodes.clear();
            nodeStates.clear();

            // initialize the state of agents on nodes
            for (unsigned int i = 0; i < agents.size(); i++) {
                for (const auto &label: solver->solution->plans[i]->path) {
                    nodes[label.nodeId].emplace_back(label.state, i);
                }
            }
            for (auto &node: nodes) {
                nodeStates[node.first] = 0;
                std::sort(node.second.begin(), node.second.end());
            }
        }

        discreteFixedDelay.update(currentTimestep, delayStart, delayInterval);

        currentTimestep++;

        if (outputFile.is_open()) {
            outputFile << currentTimestep << std::endl;
        }

        if (debug) {
            SPDLOG_DEBUG("begin timestep {}", currentTimestep);
            std::ostringstream oss;
            for (unsigned int i = 0; i < agents.size(); i++) {
                oss.str("");
                oss.clear();
                oss << "agent " << i << " (" << agents[i].start << "->" << agents[i].goal << "): ";
                for (const auto &label: solver->solution->plans[i]->path) {
                    oss << "(" << label.state << "," << label.nodeId << ")->";
                }
                SPDLOG_DEBUG("{}", oss.str());
            }
        }



/*        if ((pauseTimestep == 0 && delayInterval > 0) || currentTimestep == pauseTimestep) {
            updateDelayedSet(currentTimestep, pauseTimestep == 0);
        }
        if (pauseTimestep > 0 && currentTimestep == pauseTimestep) {
            updateDelayedSet(currentTimestep);
        }
        updateDelayedSet(currentTimestep);*/
        // advance the state of the agents if the next state shares nodeId with current state
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            const auto &label = solver->solution->plans[i]->path[state];
            while (state + 1 < solver->solution->plans[i]->path.size()) {
                const auto &nextLabel = solver->solution->plans[i]->path[state + 1];
                if (label.nodeId == nextLabel.nodeId) {
                    if (debug) {
                        SPDLOG_DEBUG("agent {}: ({},{}) -> ({},{})", i, state, label.nodeId, state + 1, nextLabel.nodeId);
//                        std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") -> ("
//                                  << state + 1 << "," << nextLabel.nodeId << ")" << std::endl;
                    }
                    ++nodeStates[label.nodeId];
                    ++state;
                } else break;
            }
        }

        int count = 0;
        std::vector<unsigned int> unblocked;

        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            if (state + 1 >= solver->solution->plans[i]->path.size()) {
                if (debug) {
                    SPDLOG_DEBUG("agent {}: completed", i);
//                    std::cout << "agent " << i << ": completed" << std::endl;
                }
                agentOutputStates[i] = "complete";
                agents[i].blocked = true;
                agents[i].delayed = false;
                ++count;
                continue;
            }
            agents[i].timestep = currentTimestep;
            const auto &label = solver->solution->plans[i]->path[state];
            const auto &nextLabel = solver->solution->plans[i]->path[state + 1];
            const auto &nextNode = nodes[nextLabel.nodeId];
            auto nextNodeState = nodeStates[nextLabel.nodeId];

//            std::cout << label.nodeId << " " << nextLabel.nodeId << " " << nextNode.size() << std::endl;
//            if (label.nodeId == nextLabel.nodeId) {
//                nextNodeState++;
//            }

            assert(nextNodeState < nextNode.size());
            auto next = nextNode[nextNodeState];
            if (next.first == nextLabel.state && next.second == i) {
                unblocked.emplace_back(i);
            } else {
                // blocked
                if (debug) {
                    SPDLOG_DEBUG("agent {}: ({},{}) blocked", i, state, label.nodeId);
//                    std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") blocked"
//                              << std::endl;
                }
                if (!agents[i].blocked) {
                    std::cerr << "error" << std::endl;
                    exit(-1);
                }
                agentOutputStates[i] = "block";
                agents[i].blocked = true;
                agents[i].delayed = false;
//                continue;
            }
//            if (debug) {
//                std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << std::endl;
//            }
        }

        bool needReplan = false;

        for (auto i: unblocked) {
            auto &state = agents[i].state;
            const auto &label = solver->solution->plans[i]->path[state];
            const auto &nextLabel = solver->solution->plans[i]->path[state + 1];
            const auto &nextNode = nodes[nextLabel.nodeId];

            // delay by agent
            if (delayType == "agent" && delayedSet.find(i) != delayedSet.end()) {
                if (debug) {
                    SPDLOG_DEBUG("agent {}: ({},{}) delayed by agent", i, state, label.nodeId);
//                    std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") delayed by agent"
//                              << std::endl;
                }
                agentOutputStates[i] = "delay";
                agents[i].blocked = false;
                agents[i].delayed = true;
                needReplan = true;
                continue;
            }

            // delay by edge
            auto &edge = graph.getEdge(label.nodeId, nextLabel.nodeId);
            if (delayType == "edge" && delayedSet.find(edge.index) != delayedSet.end()) {
                if (debug) {
                    SPDLOG_DEBUG("agent {}: ({},{}) delayed by edge", i, state, label.nodeId);
//                    std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") delayed by edge"
//                              << std::endl;
                }
                agentOutputStates[i] = "delay";
                agents[i].blocked = false;
                agents[i].delayed = true;
                needReplan = true;
                continue;
            }

/*            if (label.nodeId != nextLabel.nodeId) {
//                auto &edge = graph.getEdge(label.nodeId, nextLabel.nodeId);
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
                    agents[i].blocked = false;
                    continue;
                }
            }*/
/*            if (outputFile.is_open()) {
                auto &node = graph.getNode(nextLabel.nodeId);
                outputFile << i << " " << node.index << " " << node.x << " " << node.y << std::endl;
            }*/
            if (debug) {
                SPDLOG_DEBUG("agent {}: ({},{}) -> ({},{})", i, state, label.nodeId, state + 1, nextLabel.nodeId);
//                std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") -> ("
//                          << state + 1 << "," << nextLabel.nodeId << ")" << std::endl;
            }
            agentOutputStates[i] = "move";
            agents[i].blocked = true;
            agents[i].delayed = false;
            agents[i].current = nextLabel.nodeId;
//            agents[i].waitingTimestep = 0;
            ++state;
            ++nodeStates[label.nodeId];

            if (state + 1 >= solver->solution->plans[i]->path.size()) {
                if (firstAgentArrivingTimestep == 0) {
                    firstAgentArrivingTimestep = currentTimestep;
                }
                ++count;
            }

        }

        if (outputFile.is_open()) {
            for (unsigned int i = 0; i < agents.size(); i++) {
                auto &node = graph.getNode(agents[i].current);
//                auto delayed = delayedSet.find(i) != delayedSet.end();
                outputFile << i << " " << node.index << " " << (agents[i].blocked ? "node" : "edge") << " " << agentOutputStates[i] << std::endl;
            }
        }


//        std::cout << count << std::endl;

        for (size_t i = 0; i < agents.size(); i++) {
            agentRealPaths[i].emplace_back(agents[i].current);
        }

        if (count >= agents.size()) {
            break;
        }


        if (replanMode) {
            if (needReplan) {
                timestepJson["replan"] = true;
                auto currentExecutionTime = replan();
                if (currentExecutionTime < 0) {
                    timestepJson["success"] = false;
                    if (!replanNonstop) {
                        currentTimestep = maxTimeStep + 100;
                        break;
                    }
                } else {
                    timestepJson["success"] = true;
                    if (firstAgentArrivingTimestep == 0) {
                        firstAgentArrivingExecutionTime += currentExecutionTime;
                    }
                    executionTime += currentExecutionTime;
                    refresh = true;
                }
            } else {
                timestepJson["replan"] = false;
                timestepJson["success"] = true;
            }

        }

        writeTimestepOutput();
    }
    writeTimestepOutput();
//    std::cout << "window " << window << ": " << averageMakeSpan() << std::endl;

/*    bool unfinish = false;
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (agents[i].current != agents[i].goal) {
            unfinish = true;
//            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << ", goal " << agents[i].goal << std::endl;
            break;
        }
    }*/

    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].start = savedStart[i];
    }

    closeOutputFiles();

    if (currentTimestep > maxTimeStep + 1) {
        return 0;
    }

    return countCompletedAgents();
}