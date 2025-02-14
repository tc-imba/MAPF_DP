//
// Created by liu on 3/1/2022.
//

#include "ContinuousDefaultSimulator.h"
#include <iostream>
#include <random>

unsigned int ContinuousDefaultSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                                                  unsigned int delayStart, unsigned int delayInterval) {
    //    if (!success) return true;
    std::unordered_map<unsigned int, std::vector<std::pair<unsigned int, unsigned int>>> nodes;
    //    std::vector<unsigned int> agentStates(agents.size(), 0);
    std::unordered_map<unsigned int, int> nodeStates;
//    std::vector<unsigned int> savedStart(agents.size());

    initDelayedIntervals();
    openOutputFiles();
    outputPaths.clear();
    outputPaths.resize(agents.size());

    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].state = 0;
        agents[i].blocked = true;
        agents[i].delayed = false;
//        savedStart[i] = agents[i].current = agents[i].start;
    }

//    depGraph.solver = solver;
//    depGraph.init();
//    depGraph.feasibilityCheckTest(false, true);
//    depGraph.addSavedEdges();

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
            outputFile << i << " " << node.index << " " << node.x << " " << node.y << std::endl;
        }
    }


    while (currentTimestep < maxTimeStep) {
        if (outputFile.is_open()) {
            outputFile << currentTimestep << std::endl;
        }

        SPDLOG_DEBUG("begin timestep {}", currentTimestep);

        if (debug) {
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

        /*        if (currentTimestep >= delayStart && (size_t) (currentTimestep - delayStart) % delayInterval == 0) {
            updateDelayedSet(currentTimestep);
//            delayedSet.clear();
        }*/

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
                        printAgent(i, "same node");
                    }
                    ++nodeStates[label.nodeId];
                    ++state;
                } else
                    break;
            }
        }

        int count = 0;
        std::set<unsigned int> unblocked;

        double newCurrentTimestep = currentTimestep;
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            const auto &label = solver->solution->plans[i]->path[state];
            if (state + 1 < solver->solution->plans[i]->path.size() && !agents[i].blocked &&
                agents[i].arrivingTimestep <= currentTimestep) {
                // unblocked -> blocked (moved)
                newCurrentTimestep = std::max(newCurrentTimestep, agents[i].arrivingTimestep);
                const auto &nextLabel = solver->solution->plans[i]->path[state + 1];
                auto currentNodeId = label.nodeId;
                auto nextNodeId = nextLabel.nodeId;

                if (outputFile.is_open()) {
                    outputFile << i << " " << graph.getNode(nextNodeId).index << std::endl;
                }
                if (debug) {
                    printAgent(i, "");
                }
                //                nodeAgentMap.erase(currentNodeId);
                agents[i].current = nextNodeId;
                agents[i].timestep = agents[i].arrivingTimestep;
                agents[i].blocked = true;
                agents[i].delayed = false;
                ++state;
                ++nodeStates[label.nodeId];
            }
            if (state + 1 < solver->solution->plans[i]->path.size()) {
                const auto &nextLabel = solver->solution->plans[i]->path[state + 1];
                const auto &nextNode = nodes[nextLabel.nodeId];
                auto nextNodeState = nodeStates[nextLabel.nodeId];
                assert(nextNodeState < nextNode.size());
                auto next = nextNode[nextNodeState];
                if (next.first == nextLabel.state && next.second == i) {
                    unblocked.emplace(i);
                } else {
                    // must be blocked
                    if (!agents[i].blocked) {
                        std::cerr << "error" << std::endl;
                        exit(-1);
                    }
                }
            }
        }
        currentTimestep = newCurrentTimestep;
        SPDLOG_DEBUG("expanded timestep {}", currentTimestep);

        /*        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            if (state + 1 >= solver->solution->plans[i]->path.size()) {
                if (debug) {
                    printAgent(i, "completed");
                }
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
                    std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") blocked"
                              << std::endl;
                }
                if (!agents[i].blocked) {
                    std::cerr << "error" << std::endl;
                    exit(-1);
                }
                agents[i].blocked = true;
                agents[i].delayed = false;
//                continue;
            }
            if (debug) {
                std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << std::endl;
            }
        }*/

        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            if (state + 1 >= solver->solution->plans[i]->path.size()) {
                ++count;
                if (firstAgentArrivingTimestep == 0) {
                    firstAgentArrivingTimestep = agents[i].arrivingTimestep;
                }
                if (debug) {
                    printAgent(i, "completed");
                }
                continue;
            }

            if (unblocked.find(i) == unblocked.end()) {
                if (debug) {
                    printAgent(i, "blocked");
                }
                continue;
            }

            const auto &label = solver->solution->plans[i]->path[state];
            const auto &nextLabel = solver->solution->plans[i]->path[state + 1];
            const auto &nextNode = nodes[nextLabel.nodeId];
            auto &edge = graph.getEdge(label.nodeId, nextLabel.nodeId);

            if (agents[i].blocked) {
                /*Graph::NodeEdgeState state1{false, sqrt(2) / 4, label.nodeId, nextLabel.nodeId};
                bool conflicted = false;
                for (unsigned int j = 0; j < agents.size(); j++) {
                    if (i == j) continue;
                    Graph::NodeEdgeState state2;
                    if (agents[j].blocked) {
                        state2 = Graph::NodeEdgeState{true, sqrt(2) / 4, solver->solution->plans[j]->path[agents[j].state].nodeId, 0};
                    } else {
                        state2 = Graph::NodeEdgeState{false, sqrt(2) / 4, solver->solution->plans[j]->path[agents[j].state].nodeId, solver->solution->plans[j]->path[agents[j].state + 1].nodeId};
                    }
                    SPDLOG_DEBUG("{} {} {} {}", state1.srcNodeId, state1.destNodeId, state2.srcNodeId, state2.destNodeId);
                    if (graph.isConflict(state1, state2)) {
                        conflicted = true;
                        break;
                    }
                }
                if (conflicted) {
                    if (debug) {
                        printAgent(i, "blocked");
                    }
                    agents[i].delayed = false;
                    continue;
                }*/
                // blocked -> unblocked
                agents[i].blocked = false;
                agents[i].arrivingTimestep = getAgentArrivingTime(currentTimestep, delayInterval, i, edge);
                if (agents[i].arrivingTimestep - currentTimestep > edge.length) {
                    agents[i].delayed = true;
                }
                arrivingTimestepSet.insert(agents[i].arrivingTimestep);
            } else {
                // already unblocked, do nothing
            }
            if (debug) {
                printAgent(i, "unblocked");
            }
            /*            // delay by agent
            if (delayType == "agent" && delayedSet.find(i) != delayedSet.end()) {
                if (debug) {
                    std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") delayed by agent"
                              << std::endl;
                }
                agents[i].blocked = false;
                agents[i].delayed = true;
                continue;
            }

            // delay by edge
            auto &edge = graph.getEdge(label.nodeId, nextLabel.nodeId);
            if (delayType == "edge" && delayedSet.find(edge.index) != delayedSet.end()) {
                if (debug) {
                    std::cout << "agent " << i << ": (" << state << "," << label.nodeId << ") delayed by edge"
                              << std::endl;
                }
                agents[i].blocked = false;
                agents[i].delayed = true;
                continue;
            }*/

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


            /*            agents[i].blocked = true;
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
            }*/
        }

        if (count >= agents.size()) {
            break;
        }

        //        std::cout << count << std::endl;

        advanceTimestep(currentTimestep);

        if (replanMode) {
            //            std::cout << "replan: " << currentTimestep << std::endl;
            auto currentExecutionTime = replan();
            if (currentExecutionTime < 0) {
                currentTimestep = maxTimeStep + 100;
                break;
            }
            bool firstAgentArrived = firstAgentArrivingTimestep > 0;
            if (!firstAgentArrived) {
                firstAgentArrivingExecutionTime += currentExecutionTime;
            }
            executionTime += currentExecutionTime;
//            executionTimeVec.emplace_back(firstAgentArrived, currentExecutionTime);
            refresh = true;
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

/*    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].start = savedStart[i];
    }*/

    closeOutputFiles();

    if (currentTimestep > maxTimeStep + 1) {
        return 0;
    }

    return countCompletedAgents();
}
