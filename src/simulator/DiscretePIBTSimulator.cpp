//
// Created by liuyh on 29/4/2023.
//

#include "DiscretePIBTSimulator.h"
#include <boost/filesystem.hpp>

unsigned int DiscretePIBTSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                                    unsigned int delayStart, unsigned int delayInterval) {
    initPIBTVariables();

//    for (const auto& a: A) {
//        std::cout << a->getMode() << " " << a->getHead() << " " << a->getTail() << " " << a->getGoal() << std::endl;
//    }

    auto start = std::chrono::steady_clock::now();
    int num_activate = 0;
    bool flg_stop = false;

    executionTimeStart = std::chrono::steady_clock::now();
    while (!flg_stop) {
        SPDLOG_DEBUG("begin timestep {}", currentTimestep);
        updateDelayedSet(currentTimestep, delayStart, delayInterval);

        pibt::Agents unstable;

        // step 1, check transition
        if (debug) {
            std::cerr << "step 1, check transition" << std::endl;
        }
        for (const auto &a: A) {
            if (a->getMode() != pibt::Agent::EXTENDED) {
                unstable.push_back(a.get());
            } else {  // mode == extended

                // skip according to probability
                unsigned int i = std::distance(A.begin(), std::find(A.begin(), A.end(), a));
                if (delayedSet.find(i) != delayedSet.end()) continue;
//                if (getRandomFloat(0, 1, MT) <= delayProbs[i]) continue;

                a->activate();
                unstable.push_back(a.get());

                // check termination (maximum step)
                ++num_activate;
                if (num_activate >= max_activation) {
                    flg_stop = true;
                    break;
                }
            }
        }

        if (flg_stop) break;

        // register all
        pibt::States config;
        for (const auto &a: A) config.push_back(a->getState());
        exec.push_back(config);

        // step 2, check termination
        if (debug) {
            std::cerr << "step 2, check termination" << std::endl;
        }
        bool solved = true;
        for (const auto &a: A) {
            if (a->getMode() != pibt::Agent::CONTRACTED ||
                a->getTail() != a->getGoal()) {
                solved = false;
                break;
            } else if (firstAgentArrivingTimestep == 0) {
                firstAgentArrivingTimestep = currentTimestep;
            }
        }
        if (solved) {
            break;
        }

        // step 3, interaction
        if (debug) {
            std::cerr << "step 3, interaction" << std::endl;
        }
        do {
            while (!unstable.empty()) {
                pibt::Agent *a = randomChoose(unstable, MT.get());

                // activate
                a->activate();
                ++num_activate;
                if (num_activate >= max_activation) {
                    unstable.clear();
                    flg_stop = true;
                    break;
                }

                if (a->isStable()) {
                    // remove from unstable
                    auto itr = std::find(unstable.begin(), unstable.end(), a);
                    unstable.erase(itr);
                }
            }

            // check again whether agents are in stable
            if (!flg_stop) {
                for (const auto &a: A) {
                    if (!a->isStable()) unstable.push_back(a.get());
                }
            }
        } while (!unstable.empty());

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (firstAgentArrivingTimestep == 0) {
            firstAgentArrivingExecutionTime += elapsed_seconds.count();
        }
        executionTime += elapsed_seconds.count();
        start = end;
        currentTimestep++;
        saveExecutionTime();
    }
    saveExecutionTime();
    convertResult();
    return countCompletedAgents();
}

void DiscretePIBTSimulator::initPIBTVariables() {
    std::string mapFileName = graph.getFileName() + ".map";
    G = std::make_shared<pibt::Grid>(mapFileName);
    MT = std::make_shared<std::mt19937>(0);

//    std::cout << pibt::Agent::ST_DIAGRAM.size() << std::endl;
//    std::cout << pibt::Agent::CONFIGURATIONS.size() << std::endl;
//    for (auto &v : pibt::Agent::ST_DIAGRAM) {
//        if (!v.empty()) {
//            std::cout << v.back()->id << std::endl;
//        }
//    }
//    std::set<pibt::State *> test;
//    for (auto s: pibt::Agent::ST_DIAGRAM) test.emplace(s);
//    for (auto s: pibt::Agent::CONFIGURATIONS) test.emplace(s);
    pibt::Agent::reset();
    pibt::Agent::setMT(MT.get());
    pibt::Agent::setVerbose(debug);
    pibt::Agent::setG(G.get());

    // convert the paths of agents into configs
    pibt::Configs configs;
    size_t maxTimestep = 0;
    for (auto &plan: solver->solution->plans) {
        maxTimestep = std::max(maxTimestep, plan->path.size());
    }
    for (size_t timestep = 0; timestep < maxTimestep; timestep++) {
        pibt::Config config;
        for (size_t i = 0; i < solver->solution->plans.size(); i++) {
            auto &plan = solver->solution->plans[i];
            if (timestep < plan->path.size()) {
                auto &node = graph.getNode(plan->path[timestep].nodeId);
                // x-y axis reversed
                if (!G->existNode(node.y, node.x)) {
                    pibt::halt("node does not exist");
                }
                config.push_back(G->getNode(node.y, node.x));
            } else {
                config.push_back(configs.back()[i]);
            }
        }
        configs.emplace_back(std::move(config));
    }

    for (size_t i = 0; i < agents.size(); i++) {
        pibt::Path path;
        for (auto c: configs) path.push_back(c[i]);
//        std::shared_ptr<pibt::Agent> agent = std::make_shared<pibt::CausalPIBT_MAPF>(path);
        std::shared_ptr<pibt::Agent> agent = std::make_shared<pibt::CausalPIBT_MAPF>(path);
        agent->init(path.front(), path.back());
        A.emplace_back(std::move(agent));
    }

    max_activation = 100000;
}

bool DiscretePIBTSimulator::checkSolved() {
    for (const auto &a: A) {
        if (a->getMode() != pibt::Agent::CONTRACTED ||
            a->getTail() != a->getGoal()) {
            return false;
        } else if (firstAgentArrivingTimestep == 0) {
            firstAgentArrivingTimestep = exec.size() - 1;
        }
    }
    return true;
}


void DiscretePIBTSimulator::convertResult() {
    auto makespan = exec.size() - 1;
    for (size_t i = 0; i < agents.size(); i++) {
        auto node = A[i]->getTail();
        auto x = node->id % G->getWidth();
        auto y = node->id / G->getWidth();
        // x-y axis reversed
        auto nodeId = graph.getNodeIdByGridPos(y, x);
        agents[i].current = nodeId;

        auto cost = makespan;
        auto s = exec[cost - 1][i];
        while (s->tail == A[i]->getGoal() && cost > 0) {
            --cost;
            s = exec[cost - 1][i];
        }
        agents[i].timestep = (double) cost;
    }

    openOutputFiles();

    if (outputFile.is_open()) {
        for (size_t t = 0; t < exec.size(); t++) {
            outputFile << t << std::endl;
            for (size_t i = 0; i < agents.size(); i++) {
                auto s = exec[t][i];
                auto nodeId = graph.getNodeIdByGridPos(s->tail->pos.y, s->tail->pos.x);
                auto &node = graph.getNode(nodeId);
                outputFile << i << " " << node.index << " " << node.x << " " << node.y << std::endl;
            }
        }
        closeOutputFiles();
    }


}

