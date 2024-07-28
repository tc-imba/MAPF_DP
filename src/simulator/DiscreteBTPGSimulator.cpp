//
// Created by liuyh on 5/3/2024.
//

#include "DiscreteBTPGSimulator.h"
#include <boost/filesystem.hpp>

unsigned int DiscreteBTPGSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep, unsigned int delayStart,
                                             unsigned int delayInterval) {
    initBTPGVariables();
    //    int result = sim->Simulate(btpg.get());

    sim->mode = 1;
    sim->btpg = btpg.get();
//    srand(sim->seed);
    // 1a. Initialize generated Path
    for (int i = 0; i < sim->btpg->getNumAgents(); ++i) {
        std::vector<btpg::Coord> path;
        path.push_back(sim->btpg->getAgent(i)->Type1Next->coord);
        sim->BTPGGeneratedPath.push_back(path);
    }

    // 1b.Initialize visited nodes
    std::vector<std::vector<bool>> visited;
    for (int i = 0; i < sim->btpg->getNumAgents(); ++i) {
        std::vector<bool> visitedAgent;
        for (int j = 0; j < sim->btpg->getAgent(i)->pathLength; ++j) {
            if (j == 0) visitedAgent.push_back(true);
            else
                visitedAgent.push_back(false);
        }
        visited.push_back(visitedAgent);
    }

    // 1c. Initialize finished Agent list
    std::vector<bool> finishedAgent;
    for (int i = 0; i < sim->btpg->getNumAgents(); ++i) { finishedAgent.push_back(false); }

    // 1d. Initialize robot stop numbers
    std::unordered_map<int, int> robotStopNumbers;
    for (int i = 0; i < sim->DelayedRobots.size(); ++i) { robotStopNumbers[sim->DelayedRobots[i]] = 0; }

// 2. Start simulation
#ifdef DEBUG
    std::cout << "Start BTPG simulation" << std::endl;
#endif
    sim->BTPGTotalTimeStep = 0;
    executionTimeStart = std::chrono::steady_clock::now();
    while (std::find(finishedAgent.begin(), finishedAgent.end(), false) != finishedAgent.end()) {
        auto start = std::chrono::steady_clock::now();
        sim->BTPGTotalTimeStep++;
        // 2a.Decide which agent can move at this timestep
        std::vector<int> movableAgents;
        decideMovableAgents(movableAgents, sim->BTPGTotalTimeStep - 1, delayStart, delayInterval);
//        sim->DecideMovableAgents(movableAgents, robotStopNumbers);
        // print out all the movable agents
        sim->SimulateTimeStep(movableAgents, visited, finishedAgent);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (firstAgentArrivingTimestep == 0) {
            firstAgentArrivingExecutionTime += elapsed_seconds.count();
            if (std::find(finishedAgent.begin(), finishedAgent.end(), true) != finishedAgent.end()) {
                firstAgentArrivingTimestep = sim->BTPGTotalTimeStep;
            }
        }
        executionTime += elapsed_seconds.count();
        writeTimestepOutput();
    }
#ifdef DEBUG
    std::cout << "Finish BTPG simulation" << std::endl;
#endif
    sim->CheckBTPGPathValidity();
#ifdef DEBUG
    std::cout << "Finish checking path validity" << std::endl;
#endif
//    sim->GetStatistics();

//    return 1;


    if (btpg->finish) {
        for (int i = 0; i < agents.size(); i++) {
            agents[i].current = agents[i].goal;
            agents[i].timestep = btpg->getAgent(i)->BTPGFinishedTime;
        }
        return agents.size();
    } else {
        return 0;
    }
}

void DiscreteBTPGSimulator::initBTPGVariables() {
    using namespace boost::filesystem;

    path ph = path("btpg_results") / unique_path();
    create_directories(ph);
    std::string planFileName = (ph / "plans.txt").string();

    std::ofstream fout(planFileName);
    for (size_t i = 0; i < agents.size(); i++) {
        fout << "Agent " << i << ": ";
        auto &plan = solver->solution->plans[i];
        for (auto &label: plan->path) {
            auto &node = graph.getNode(label.nodeId);
            fout << "(" << node.y << "," << node.x << ")->";
        }
        fout << std::endl;
    }
    fout.close();

    auto start = std::chrono::steady_clock::now();
    btpg = std::make_shared<btpg::BTPG>(planFileName, algorithmIdx, timeInterval);
    sim = std::make_shared<btpg::Sim>(seed, btpg->getNumAgents());
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
//    firstAgentArrivingExecutionTime += elapsed_seconds.count();
//    executionTime += elapsed_seconds.count();
    remove_all(ph);
}

void DiscreteBTPGSimulator::decideMovableAgents(std::vector<int> &movableAgents, unsigned int currentTimestep,
                                                unsigned int delayStart, unsigned int delayInterval) {
    updateDelayedSet(currentTimestep, delayStart, delayInterval);
    for (int i = 0; i < agents.size(); i++) {
        if (delayedSet.find(i) == delayedSet.end()) {
            movableAgents.emplace_back(i);
        }
    }
}