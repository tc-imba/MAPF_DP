//
// Created by liuyh on 27/11/2024.
//

#include "DiscreteSESSimulator.h"
#include "DiscreteDefaultSimulator.h"
#include "../solver/SESSolver.h"

#include <boost/filesystem.hpp>


/*void print_for_replanning(ses::ADG &adg, std::vector<int> states, std::ofstream &outFile_path) {
    using namespace ses;

    outFile_path << "version 1\n";
    int agentCnt = get_agentCnt(adg);
    for (int agent = 0; agent < agentCnt; agent++) {
        outFile_path << "4	random-32-32-10.map	32	32	";
        Location l = get_state_target(adg, agent, states[agent]);
        Location e = get_state_target(adg, agent, get_stateCnt(adg, agent) - 1);
        outFile_path << get<1>(l) << "	" << get<0>(l) << "	" << get<1>(e) << "	" << get<0>(e) << "	18.72792206\n";
    }
}*/

int DiscreteSESSimulator::step_wdelay(bool *delay_mark,
                                      std::vector<int> &delayed_agents) {
    using namespace ses;
    using namespace std;

    int agentCnt = get_agentCnt(_simulator->adg);

    //    random_device rd;
    //    mt19937 gen(rd());
    //    uniform_int_distribution<> distrib(1, 1000);

    vector<int> movable(agentCnt, 0);
    vector<int> haventStop(agentCnt, 0);
    int timeSpent = _simulator->checkMovable(movable, haventStop);
    int moveCnt = 0;

    const auto &delayedSet = discreteFixedDelay.getDelayedSet();

    for (int agent = 0; agent < agentCnt; agent++) {
        if (haventStop[agent] == 1) {
            if (delayedSet.find(agent) != delayedSet.end()) {
                delayed_agents[agent] = 1;
                //                std::cerr << "agent " << agent << ": delayed" << std::endl;
                *delay_mark = true;
            }
        }
    }

    /*for (int agent = 0; agent < agentCnt; agent++) {
        if (haventStop[agent] == 1) {
            if (distrib(gen) <= p) {
                delayed_agents[agent] = 1;
                *delay_mark = true;
            }
        }
    }*/

    if (*delay_mark) { return 0; }

    /*for (int agent = 0; agent < agentCnt; agent++) {
        if (movable[agent] == 1) {
            _simulator->states[agent] += 1;
            moveCnt += 1;
        }
    }
    if (moveCnt == 0 && timeSpent != 0) { return -1; }*/
    return timeSpent;
}

/*int _SESSimulator::simulate_wdelay(const std::string &stdoutFileName, std::ofstream &outFile_setup, int timeout) {
    using namespace ses;
    using namespace std;
    using namespace chrono;

    int agentCnt = get_agentCnt(adg);
    int stepSpend = 1;
    bool delay_mark = false;


    discreteSesSimulator.discreteFixedDelay.update(currentTimestep, delayStart, delayInterval);
    while (stepSpend != 0) {

        std::cerr << "timestep: " << currentTimestep << std::endl;
        vector<int> delayed_agents(agentCnt, 0);
        stepSpend = step_wdelay(&delay_mark, delayed_agents);

        //        std::cerr << delay_mark << std::endl;

        if (delay_mark)// a delay just happened
        {
            std::cerr << "delayed" << std::endl;
            //            print_for_replanning(adg, states, outFile_path);
            //            outFile_path.close();
            // int delayed_state = states[delayed_agent];

            int input_sw_cnt;

            //            microseconds timer_constructADG(0);
            //            auto start = high_resolution_clock::now();
            ADG adg_delayed = construct_delayed_ADG(adg, 1, 1, delayed_agents, states, &input_sw_cnt, outFile_setup);
            //            outFile_setup.close();
            //            auto stop = high_resolution_clock::now();
            //            timer_constructADG += duration_cast<microseconds>(stop - start);

            //            ADG adg_delayed_copy = copy_ADG(adg_delayed);
            //            set_switchable_nonSwitchable(get<0>(adg_delayed_copy));
            //            Simulator simulator_original(adg_delayed_copy);
            //            int originalTime = simulator_original.print_soln();
            //            Simulator simulator_ori_trunc(adg_delayed_copy, states);
            //            int oriTime_trunc = simulator_ori_trunc.print_soln();

            //            ADG adg_delayed_slow = copy_ADG(adg_delayed);

            //            microseconds timer(0);
            //            start = high_resolution_clock::now();
            Astar search(timeout, true);
            ADG replanned_adg = search.startExplore(adg_delayed, input_sw_cnt);
            //            stop = high_resolution_clock::now();
            //            timer += duration_cast<microseconds>(stop - start);

            //            if (duration_cast<seconds>(timer).count() >= timeout) {
            //                outFile << timer.count() << "," << timer.count() + timer_constructADG.count() << ",,,,,";
            //                search.print_stats(outFile);
            //                exit(0);
            //            }

            //            Simulator simulator_res(replanned_adg);
            //            int timeSum = simulator_res.print_soln(stdoutFileName.c_str());
            //            Simulator simulator_res_trunc(replanned_adg, states);
            //            int timeSum_trunc = simulator_res_trunc.print_soln();

            //            outFile << timer.count() << "," << timer.count() + timer_constructADG.count() << "," << originalTime << ","
            //                    << timeSum << "," << oriTime_trunc << "," << timeSum_trunc << ",";

            //            search.print_stats(outFile);

            //            microseconds timer_slow(0);
            //            start = high_resolution_clock::now();
            //            Astar search_slow(timeout, false);
            //            ADG replanned_slow = search_slow.startExplore(adg_delayed_slow, input_sw_cnt);
            //            stop = high_resolution_clock::now();
            //            timer_slow += duration_cast<microseconds>(stop - start);
            //
            //            if (duration_cast<seconds>(timer_slow).count() >= timeout) {
            //                outFile_slow << timer_slow.count() << "," << timer_slow.count() + timer_constructADG.count() << ",,,,,";
            //                search_slow.print_stats(outFile_slow);
            //                exit(0);
            //            }

            //            Simulator simulator_slow(replanned_slow);
            //            timeSum = simulator_slow.print_soln();
            //            Simulator simulator_slow_trunc(replanned_slow, states);
            //            timeSum_trunc = simulator_slow_trunc.print_soln();

            //            outFile_slow << timer_slow.count() << "," << timer_slow.count() + timer_constructADG.count() << ","
            //                         << originalTime << "," << timeSum << "," << oriTime_trunc << "," << timeSum_trunc << ",";
            //
            //            search_slow.print_stats(outFile_slow);

            adg = replanned_adg;
            //            print_soln(stdoutFileName.c_str());
            stepSpend = step(false);
            //            return 0;
            //          exit(0);
        }

        if (stepSpend != 0) {
            discreteSesSimulator.discreteFixedDelay.update(currentTimestep, delayStart, delayInterval);
            currentTimestep++;
        }

        if (stepSpend < 0) return -1;// stuck
    }

    Simulator simulator_res(adg);
    simulator_res.print_soln(stdoutFileName.c_str());

    //    std::cerr << "no delay happened\n";
    return 0;
}*/


unsigned int DiscreteSESSimulator::simulate(double &currentTimestep,
                                            unsigned int maxTimeStep,
                                            unsigned int delayStart,
                                            unsigned int delayInterval) {
    using namespace boost::filesystem;

    auto savedCurrentTimestep = currentTimestep;

    path ph = path("ses_results") / unique_path();
    create_directories(ph);

    std::string stdoutFileName = (ph / "out.txt").string();
    std::string adgFileName = (ph / "path.txt").string();
    std::string outFileName_setup = (ph / "delay_setup.txt").string();

    std::ofstream adgFile(adgFileName);
    std::ofstream outFile_setup(outFileName_setup);

    for (unsigned int i = 0; i < agents.size(); i++) {
        adgFile << "Agent " << i << ": ";
        for (const auto &label: solver->solution->plans[i]->path) {
            const auto &node = graph.getNode(label.nodeId);
            adgFile << "(" << (unsigned int) node.x << "," << (unsigned int) node.y
                    << ")->";
        }
        adgFile << std::endl;
    }
    adgFile.close();


    int timeout = 90;
    ses::ADG _adg = ses::construct_ADG((char *) adgFileName.c_str());
    _simulator = std::make_shared<ses::Simulator>(_adg);

    int agentCnt = ses::get_agentCnt(_simulator->adg);
    int stepSpend = 1;
    bool delay_mark = false;

    discreteFixedDelay.update(currentTimestep, delayStart, delayInterval);

    while (stepSpend != 0) {
        //        std::cerr << "timestep: " << _currentTimestep << std::endl;

        if (verboseOutput) {
            timestepJson.clear();
            timestepJson["timestep"] = currentTimestep;
            executionTimeStart = std::chrono::steady_clock::now();
        }

        std::vector<int> delayed_agents(agentCnt, 0);
        stepSpend = step_wdelay(&delay_mark, delayed_agents);

        if (delay_mark) {// a delay just happened
                         //            std::cerr << "delay_mark" << std::endl;

            //            ses::ADG adg_copy = ses::copy_ADG(_simulator->adg);
            //            ses::Simulator simulator_temp(adg_copy);
            //            std::string filename = (ph / fmt::format("{}_init.txt", _currentTimestep)).string();
            //            simulator_temp.print_soln(filename.c_str());

            int input_sw_cnt;
            ses::ADG adg_delayed = ses::construct_delayed_ADG(
                    _simulator->adg, 1, 1, delayed_agents, _simulator->states,
                    &input_sw_cnt, outFile_setup);

            //            ses::ADG adg_delayed_copy = ses::copy_ADG(adg_delayed);
            //            simulator_temp = ses::Simulator(adg_delayed_copy);
            //            filename = (ph / fmt::format("{}_delay.txt", _currentTimestep)).string();
            //            simulator_temp.print_soln(filename.c_str());

            ses::Astar search(timeout, true);
            ses::ADG adg_replanned = search.startExplore(adg_delayed, input_sw_cnt);

            //            ses::ADG adg_replanned_copy = ses::copy_ADG(adg_replanned);
            //            simulator_temp = ses::Simulator(adg_replanned_copy);
            //            filename = (ph / fmt::format("{}_replan.txt", _currentTimestep)).string();
            //            simulator_temp.print_soln(filename.c_str());

            _simulator->adg = adg_replanned;
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - executionTimeStart;
        if (firstAgentArrivingTimestep == 0) {
            firstAgentArrivingExecutionTime += elapsed_seconds.count();
        }
        executionTime += elapsed_seconds.count();
        if (verboseOutput) { writeTimestepOutput(); }

        //        stepSpend = step(adgFileName);
        stepSpend = _simulator->step(false);

        /*std::unordered_map<std::pair<int, int>, int, boost::hash<std::pair<int, int>>> conflictSet;
        for (int agent = 0; agent < agentCnt; agent++) {
            auto state = _simulator->states[agent];
            auto &ori_path = (get<1>(_simulator->adg))[agent];
            auto &loc = ori_path[state].first;
            auto it = conflictSet.find(loc);
            if (it == conflictSet.end()) {
                conflictSet.emplace_hint(it, loc, agent);
            } else {
                std::cerr << "conflict: " << it->second << " " << agent << std::endl;
            }
        }*/


        /*ses::ADG adg_replanned_copy = ses::copy_ADG(_simulator->adg);
        ses::Simulator simulator_temp = ses::Simulator(adg_replanned_copy);
        std::string filename = (ph / fmt::format("{}_step.txt", _currentTimestep)).string();
        simulator_temp.print_soln(filename.c_str());*/

        currentTimestep++;

        std::vector<int> movable(agentCnt, 0);
        std::vector<int> haventStop(agentCnt, 0);
        _simulator->checkMovable(movable, haventStop);
        bool firstAgentArriving = false;
        for (int agent = 0; agent < agentCnt; agent++) {
            if (haventStop[agent] == 1) {
                agents[agent].timestep = currentTimestep;
            } else {
                firstAgentArriving = true;
            }
            auto &location =
                    get<1>(_simulator->adg)[agent][_simulator->states[agent]].first;
            agents[agent].current =
                    graph.getNodeIdByGridPos(location.first, location.second);
        }
        if (firstAgentArrivingTimestep == 0 && firstAgentArriving) {
            firstAgentArrivingTimestep = currentTimestep;
        }

        if (stepSpend != 0) {
            discreteFixedDelay.update(currentTimestep, delayStart, delayInterval);
        }

        if (stepSpend < 0 || currentTimestep > maxTimeStep) {
            break;// stuck
        }
    }

    outFile_setup.close();
    remove_all(ph);

    auto defaultAgents = agents;
    for (int agent = 0; agent < agentCnt; agent++) {
        defaultAgents[agent].timestep = 0;
        defaultAgents[agent].arrivingTimestep = 0;
        defaultAgents[agent].current = defaultAgents[agent].start;
    }
    auto defaultSimulator = DiscreteDefaultSimulator(graph, agents, seed);
    auto solver = std::make_shared<SESSolver>(graph, agents, MakeSpanType::AVERAGE, _simulator);
    if (!solver->solve()) {
        return 0;
    }
    defaultSimulator.setSolver(solver);
    defaultSimulator.debug = true;

    auto result = defaultSimulator.simulate(savedCurrentTimestep, maxTimeStep, delayStart, delayInterval);
    agents = defaultAgents;

    return result;

    /*ses::Simulator simulator_res(_simulator->adg);
    //    simulator_res.print_soln(stdoutFileName.c_str());
    //    return 0;

    int totalSpend = 0;

    std::vector<std::vector<ses::Location>> expanded_paths;
    ses::Paths &paths = get<1>(_simulator->adg);

    for (int agent = 0; agent < agentCnt; agent++) {
        std::vector<ses::Location> expanded_path;
        expanded_path.push_back(get<0>((paths[agent])[0]));
        expanded_paths.emplace_back(std::move(expanded_path));
    }

    _currentTimestep = 0;
    std::cerr << "timestep: " << _currentTimestep << std::endl;
    stepSpend = simulator_res.step(true);
    _currentTimestep++;
    while (stepSpend != 0) {
        for (int agent = 0; agent < agentCnt; agent++) {
            auto new_location = get<0>((paths[agent])[(simulator_res.states[agent])]);
            if (!((ses::same_locations(new_location, (expanded_paths[agent]).back())) &&
                  ((size_t) (simulator_res.states[agent]) ==
                   (paths[agent]).size() - 1))) {
                (expanded_paths[agent]).push_back(new_location);
            }
        }
        totalSpend += stepSpend;
        std::cerr << "timestep: " << _currentTimestep << std::endl;
        stepSpend = simulator_res.step(true);
        _currentTimestep++;
    }

    for (int agent = 0; agent < agentCnt; agent++) {
        auto &expanded_path = expanded_paths[agent];
        agents[agent].timestep = expanded_path.size() - 1;
        auto &location = expanded_path.back();
        agents[agent].current =
                graph.getNodeIdByGridPos(location.first, location.second);
    }
*/
    return countCompletedAgents();
}
int DiscreteSESSimulator::step(const std::string &adgFileName) {
    int agentCnt = ses::get_agentCnt(_simulator->adg);

    std::vector<int> movable(agentCnt, 0);
    _simulator->checkMovable(movable);
    auto &paths = get<1>(_simulator->adg);

    for (int agent = 0; agent < agentCnt; agent++) {
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
    }

    int stepSpend = _simulator->step(false);

    if (stepSpend <= 0) { return stepSpend; }

    std::vector<std::deque<ses::Location>> expanded_paths;

    for (int agent = 0; agent < agentCnt; agent++) {
        std::deque<ses::Location> expanded_path;
        expanded_path.push_back(get<0>((paths[agent])[0]));
        expanded_paths.emplace_back(std::move(expanded_path));
    }

    int totalSpend = 0;
    while (stepSpend != 0) {
        for (int agent = 0; agent < agentCnt; agent++) {
            auto new_location = get<0>((paths[agent])[(_simulator->states[agent])]);
            if (!((ses::same_locations(new_location, (expanded_paths[agent]).back())) &&
                  ((size_t) (_simulator->states[agent]) ==
                   (paths[agent]).size() - 1))) {
                (expanded_paths[agent]).push_back(new_location);
            }
        }
        totalSpend += stepSpend;
        stepSpend = _simulator->step(false);
    }

    std::ofstream outFile(adgFileName);

    for (int agent = 0; agent < agentCnt; agent++) {
        outFile << "Agent " << agent << ": ";
        auto &expanded_path = expanded_paths[agent];
        if (movable[agent]) { expanded_path.pop_front(); }
        for (auto &location: expanded_path) {
            _simulator->print_location(outFile, location);
        }
        outFile << std::endl;
    }
    outFile.close();

    ses::ADG _adg = ses::construct_ADG((char *) adgFileName.c_str());
    _simulator = std::make_unique<ses::Simulator>(_adg);

    return totalSpend;
}
