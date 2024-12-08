//
// Created by liuyh on 25/4/2023.
//

#include "DiscreteSimulator.h"

//void DiscreteSimulator::updateDelayedSet(unsigned int currentTimestep, unsigned int delayStart, unsigned int delayInterval) {
////        if (delayInterval == 0 || timestep % delayInterval == 0) {
//    if (delayInterval == 0) return;
//    if (currentTimestep < delayStart) return;
//    if ((size_t) (currentTimestep - delayStart) % delayInterval != 0) return;
//
//    std::vector<unsigned int> delayed;
//    if (delayType == "agent") {
//        delayed.resize(agents.size());
//        /*for (unsigned int i = 0; i < agents.size(); i++) {
//            if (checkComplete) {
//                auto &state = agents[i].state;
//                if (state + 1 < solution->plans[i]->path.size()) {
//                    delayed.emplace_back(i);
//                }
//            } else {
//                delayed.emplace_back(i);
//            }
//            delayed.emplace_back(i);
//        }*/
//    } else if (delayType == "edge") {
//        delayed.resize(graph.getEdgeNum());
//    }
//    std::iota(delayed.begin(), delayed.end(), 0);
//    unsigned int newSeed = combineRandomSeed(0, 0, currentTimestep, seed);
//    std::mt19937 generator(newSeed);
//    std::shuffle(delayed.begin(), delayed.end(), generator);
//    delayedSet.clear();
//    for (unsigned int i = 0; i < (unsigned int) (((double) delayed.size()) * delayRatio); i++) {
//        delayedSet.emplace(delayed[i]);
////            std::cout << ((double) delayed.size()) * delayRatio << " " << delayed[i] << " delay" << std::endl;
//    }
////        }
//}
