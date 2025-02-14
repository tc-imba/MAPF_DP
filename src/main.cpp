#include <boost/algorithm/string.hpp>
#include <boost/dll.hpp>
#include <boost/filesystem.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "Graph.h"
#include "simulator/ContinuousDefaultSimulator.h"
#include "simulator/ContinuousOnlineSimulator.h"
#include "simulator/ContinuousPIBTSimulator.h"
#include "simulator/DiscreteBTPGSimulator.h"
#include "simulator/DiscreteDefaultSimulator.h"
#include "simulator/DiscreteOnlineSimulator.h"
#include "simulator/DiscretePIBTSimulator.h"
#include "simulator/DiscreteSESSimulator.h"
#include "solver/CBSSolver.h"
#include "solver/CCBSSolver.h"
#include "solver/EECBSSolver.h"
#include "solver/LNSSolver.h"
#include "solver/IndividualAStarSolver.h"
#include "utils/config.hpp"
#include "utils/ezOptionParser.hpp"


/*std::string double_to_string(double data) {
    auto result = std::to_string(data);
    auto pos = result.length() - 1;
    while (pos > 0 && result[pos] == '0') --pos;
    if (result[pos] == '.') --pos;
    return result.substr(0, pos + 1);
}*/


int main(int argc, const char *argv[]) {
    //    Config config(argc, argv);

    spdlog::set_pattern("[%H:%M:%S %e] [%^%=8l%$] %v [%@]");
    ez::ezOptionParser optionParser;

    optionParser.overview = "Multi Agent Path Finding with Delay Probability";
    optionParser.syntax = "./MAPF_DP [OPTIONS]";
    optionParser.example =
            "./MAPF_DP --map random --map-seed 0 --agent-seed 0 --simulation-seed 0 --agents 20 --iteration 10 "
            "--simulator online --delay-start 1 --delay-ratio 0.3 --delay-interval 10 --obstacles 270 --delay agent\n";
    optionParser.footer = "";

    optionParser.add("", false, 0, 0, "Display this Message.", "-h", "--help");
    optionParser.add("", false, 0, 0, "Debug Mode", "-d", "--debug");
    optionParser.add("", false, 0, 0, "Verbose Output", "-v", "--verbose");
    optionParser.add("", false, 0, 0, "All Constraints", "--all");
    optionParser.add("", false, 0, 0, "Use delay probability (in solver)", "--dp");
    optionParser.add("", false, 0, 0, "Use naive feasibility check", "--naive-feasibility");
    optionParser.add("", false, 0, 0, "Use naive cycle check", "--naive-cycle");
    optionParser.add("", false, 0, 0, "Use fast cycle check", "--fast-cycle");
    optionParser.add("", false, 0, 0, "Use only cycle check", "--only-cycle");
    optionParser.add("", false, 0, 0, "Classify feasibility types", "--feasibility-type");
    optionParser.add("", false, 0, 0, "Use prioritized replan", "--prioritized-replan");
    optionParser.add("", false, 0, 0, "Use prioritized replan with optimization ", "--prioritized-opt");
    optionParser.add("", false, 0, 0, "Use online with optimization ", "--online-opt");
    optionParser.add("", false, 0, 0, "Don't use cache for map generator and solver", "--no-cache");
    optionParser.add("", false, 0, 0, "Use group conflicts optimization", "--group");
    optionParser.add("", false, 0, 0, "Use group by determined edges optimization", "--group-determined");
    optionParser.add("", false, 0, 0, "Use nonstop mode when replan fails", "--replan-nonstop");
    optionParser.add("random", false, 1, 0, "Map type (random / warehouse)", "-m", "--map");
    optionParser.add("map", false, 1, 0, "Map name (eg., sparse, dense, super_dense)", "--map-name");
    optionParser.add("", false, 1, 0, "Map file", "--map-file");
    optionParser.add("", false, 1, 0, "Task file", "--task-file");
    optionParser.add("", false, 1, 0, "Log file", "--log-file");
    optionParser.add("maximum", false, 1, 0, "Objective type (maximum / average)", "--objective");
    optionParser.add("default", false, 1, 0, "Simulator type (default / online / replan / pibt / snapshot)",
                     "--simulator");
    optionParser.add("continuous", false, 1, 0, "Timing type (discrete / continuous)", "--timing");
    optionParser.add("node-node,edge-edge,node-edge", false, 1, 0, "Conflict types", "--conflicts");
    optionParser.add("none", false, 1, 0,
                     "Remove redundant unsettled edges in continuous online (none / physical / graph)",
                     "--remove-redundant");
    optionParser.add("", false, 1, 0, "Statistics Output Filename", "-o", "--output");
    optionParser.add("json", false, 1, 0, "Output Format", "--output-format");
    optionParser.add("", false, 1, 0, "Simulator Output Filename", "--simulator-output");
    optionParser.add("edge", false, 1, 0, "Delay type (agent / node / edge)", "--delay");
    optionParser.add("eecbs", false, 1, 0, "Solver (default / individual / eecbs / ccbs / lns", "--solver");
    optionParser.add("", false, 1, 0, "Solver binary (for CCBS / EECBS)", "--solver-binary");
    optionParser.add("none", false, 1, 0, "(none, start, end, collision)", "--snapshot-order");
    optionParser.add("array", false, 1, 0, "(boost / array)", "--dep-graph");

    auto validWindowSize = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Window Size (0 means no limit, deprecated)", "-w", "--window", validWindowSize);

    //    auto validObstacles = new ez::ezOptionValidator("u4", "ge", "0");
    //    optionParser.add("90", false, 1, 0, "Obstacles", "--obstacles", validObstacles);

    auto validMapSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Map Seed", "--map-seed", validMapSeed);

    auto validAgentSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Agent Seed", "--agent-seed", validAgentSeed);

    auto validAgentSkip = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Skip agents skip in task file", "--agent-skip", validAgentSkip);

    auto validSimulationSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Simulation Seed", "--simulation-seed", validSimulationSeed);

    auto validAgents = new ez::ezOptionValidator("u4", "ge", "1");
    optionParser.add("10", false, 1, 0, "Agents Number", "-a", "--agents", validAgents);

    auto validIteration = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("10", false, 1, 0, "Iteration Number", "-i", "--iteration", validIteration);

    auto validMin = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0", false, 1, 0, "Min Delay Probability (in solver)", "--min", validMin);
    auto validMax = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0.5", false, 1, 0, "Min Delay Probability (in solver)", "--max", validMax);

    auto validObstacleRatio = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0.1", false, 1, 0, "Obstacle Ratio", "--obstacle-ratio", validObstacleRatio);

    auto validDelayRatio = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0.2", false, 1, 0, "Delay Ratio", "--delay-ratio", validDelayRatio);

    auto validDelayInterval = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("1", false, 1, 0, "Delay Interval", "--delay-interval", validDelayInterval);

    //    auto validDelayStart = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Delay Start", "-p", "--delay-start");

    auto validKNeighbor = new ez::ezOptionValidator("u4", "ge", "2");
    optionParser.add("2", false, 1, 0, "Max Edge Length", "--k-neighbor", validKNeighbor);

    auto validSuboptimality = new ez::ezOptionValidator("d", "ge", "1");
    optionParser.add("1", false, 1, 0, "Suboptimality of CBS", "--suboptimality", validSuboptimality);

    auto validReplanSuboptimality = new ez::ezOptionValidator("d", "ge", "1");
    optionParser.add("1", false, 1, 0, "Suboptimality of CBS in Replan", "--replan-suboptimality",
                     validReplanSuboptimality);

    auto validMaxTimestep = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("300", false, 1, 0, "Max Timestep", "--max-timestep", validMaxTimestep);

    auto validDeltaTimestep = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0", false, 1, 0, "Delta Timestep", "--delta-timestep", validDeltaTimestep);

    optionParser.parse(argc, argv);
    if (optionParser.isSet("-h")) {
        std::string usage;
        optionParser.getUsage(usage, 80, ez::ezOptionParser::ALIGN);
        std::cerr << usage;
        return 1;
    }

    std::string mapType, mapName, objective, simulatorType, timingType, conflictTypes, removeRedundant, outputFileName,
            outputFormat, simulatorOutputFileName, delayType, solverType, solverBinaryFile, mapFile, taskFile, logFile,
            snapshotOrder, topoGraphType;
    unsigned long window, mapSeed, agentSeed, agentSkip, simulationSeed, agentNum, iteration, kNeighbor;
    long delayStart, delayInterval;
    double minDP, maxDP, delayRatio, suboptimality, replanSuboptimality, deltaTimestep, maxTimestep, obstacleRatio;
    bool debug, verboseOutput, allConstraint, useDP, naiveFeasibilityCheck, naiveCycleCheck, onlyCycleCheck,
            fastCycleCheck, feasibilityType, prioritizedReplan, prioritizedOpt, onlineOpt, noCache, useGroup,
            useGroupDetermined, replanNonstop;
    optionParser.get("--map")->getString(mapType);
    optionParser.get("--map-file")->getString(mapFile);
    optionParser.get("--map-name")->getString(mapName);
    optionParser.get("--task-file")->getString(taskFile);
    optionParser.get("--log-file")->getString(logFile);
    optionParser.get("--objective")->getString(objective);
    optionParser.get("--simulator")->getString(simulatorType);
    optionParser.get("--timing")->getString(timingType);
    optionParser.get("--conflicts")->getString(conflictTypes);
    optionParser.get("--remove-redundant")->getString(removeRedundant);
    optionParser.get("--output")->getString(outputFileName);
    optionParser.get("--output-format")->getString(outputFormat);
    optionParser.get("--simulator-output")->getString(simulatorOutputFileName);
    optionParser.get("--solver")->getString(solverType);
    optionParser.get("--solver-binary")->getString(solverBinaryFile);
    optionParser.get("--snapshot-order")->getString(snapshotOrder);
    optionParser.get("--dep-graph")->getString(topoGraphType);
    optionParser.get("--window")->getULong(window);
    //    optionParser.get("--obstacles")->getULong(obstacles);
    optionParser.get("--map-seed")->getULong(mapSeed);
    optionParser.get("--agent-seed")->getULong(agentSeed);
    optionParser.get("--agent-skip")->getULong(agentSkip);
    optionParser.get("--simulation-seed")->getULong(simulationSeed);
    optionParser.get("--agents")->getULong(agentNum);
    optionParser.get("--iteration")->getULong(iteration);
    optionParser.get("--min")->getDouble(minDP);
    optionParser.get("--max")->getDouble(maxDP);
    optionParser.get("--delay")->getString(delayType);
    optionParser.get("--obstacle-ratio")->getDouble(obstacleRatio);
    optionParser.get("--delay-ratio")->getDouble(delayRatio);
    optionParser.get("--delay-interval")->getLong(delayInterval);
    optionParser.get("--delay-start")->getLong(delayStart);
    optionParser.get("--k-neighbor")->getULong(kNeighbor);
    optionParser.get("--suboptimality")->getDouble(suboptimality);
    optionParser.get("--replan-suboptimality")->getDouble(replanSuboptimality);
    optionParser.get("--max-timestep")->getDouble(maxTimestep);
    optionParser.get("--delta-timestep")->getDouble(deltaTimestep);
    debug = optionParser.isSet("--debug");
    verboseOutput = optionParser.isSet("--verbose");
    allConstraint = optionParser.isSet("--all");
    useDP = optionParser.isSet("--dp");
    naiveFeasibilityCheck = optionParser.isSet("--naive-feasibility");
    naiveCycleCheck = optionParser.isSet("--naive-cycle");
    onlyCycleCheck = optionParser.isSet("--only-cycle");
    fastCycleCheck = optionParser.isSet("--fast-cycle");
    feasibilityType = optionParser.isSet("--feasibility-type");
    prioritizedReplan = optionParser.isSet("--prioritized-replan");
    prioritizedOpt = optionParser.isSet("--prioritized-opt");
    onlineOpt = optionParser.isSet("--online-opt");
    noCache = optionParser.isSet("--no-cache");
    useGroup = optionParser.isSet("--group");
    useGroupDetermined = optionParser.isSet("--group-determined");
    replanNonstop = optionParser.isSet("--replan-nonstop");
    //    removeRedundant = optionParser.isSet("--remove-redundant");

    //    spdlog::
    std::shared_ptr<spdlog::logger> logger;
    try {
        if (!logFile.empty()) {
            logger = spdlog::basic_logger_st("file_logger", logFile, true);
        } else {
            logger = spdlog::stderr_color_mt("stderr");
        }
        logger->flush_on(spdlog::level::debug);
        spdlog::set_default_logger(logger);
    } catch (const spdlog::spdlog_ex &ex) {
        std::cerr << "Log initialization failed: " << ex.what() << std::endl;
        exit(-1);
    }
    if (debug) { spdlog::set_level(spdlog::level::debug); }
    if (window == 0) { window = INT_MAX; }
    if (delayInterval < 0) { delayInterval = INT_MAX; }
    if (delayStart < 0) { delayStart = INT_MAX; }
    bool nodeNodeConflict = conflictTypes.find("node-node") != std::string::npos;
    bool edgeEdgeConflict = conflictTypes.find("edge-edge") != std::string::npos;
    bool nodeEdgeConflict = conflictTypes.find("node-edge") != std::string::npos;

    if (topoGraphType != "boost" && topoGraphType != "array") {
        SPDLOG_ERROR("--dep-graph must be boost or array!");
        exit(-1);
    }

    if (solverBinaryFile.empty()) {
        auto solverBinaryPath = boost::dll::program_location().parent_path();
        if (solverType == "ccbs") {
            solverBinaryPath = solverBinaryPath / "Continuous-CBS" / "CCBS";
        } else if (solverType == "eecbs") {
            solverBinaryPath = solverBinaryPath / "EECBS" / "eecbs";
        } else if (solverType == "lns") {
            solverBinaryPath = solverBinaryPath / "MAPF-LNS2" / "lns";
        }
        if (!solverBinaryPath.empty()) { solverBinaryFile = solverBinaryPath.string(); }
    } else {
        auto solverBinaryPath = boost::filesystem::path(solverBinaryFile);
        solverBinaryFile = boost::filesystem::canonical(solverBinaryPath).string();
    }
    SPDLOG_INFO("solver binary file: {}", solverBinaryFile);

    std::ofstream fout;
    auto buf = std::make_unique<char[]>(4096);
    if (!outputFileName.empty()) {
        if (outputFormat == "bson") {
            fout.open(outputFileName, std::ios_base::binary);
        } else {
            fout.open(outputFileName);
        }
        fout.rdbuf()->pubsetbuf(buf.get(), 4096);
        std::cerr << "redirect output to " << outputFileName << std::endl;
    }
    std::ostream &out = fout.is_open() ? fout : std::cout;

    //    std::string lockFileName = outputFileName + ".lock";
    //    std::ofstream lockFile(lockFileName, std::ios_base::app);
    //    lockFile.close();


    Graph graph;
    unsigned int height = 32;
    unsigned int width = 32;
    unsigned int obstacles = std::min((unsigned int) (height * width * obstacleRatio), height * width);

    unsigned int deliveryWidth = 10;
    unsigned int deliveryX = 7;
    unsigned int deliveryY = 4;
    unsigned int maxX = 3 * deliveryX + 1;
    unsigned int maxY = deliveryY * (deliveryWidth + 1) + 13;

    //    unsigned int agents = 30;
    graph.initDelayProbability(minDP, maxDP);
    graph.debug = debug;
    graph.nodeNodeConflict = nodeNodeConflict;
    graph.edgeEdgeConflict = edgeEdgeConflict;
    graph.nodeEdgeConflict = nodeEdgeConflict;
    graph.noCache = noCache;

    std::string filename, cacheFileName, taskFileType;
    if (mapType == "random") {
        filename = timingType + "-random-" + std::to_string(height) + "-" + std::to_string(width) + "-" +
                   std::to_string((unsigned int) (obstacleRatio * 100)) + "-" + std::to_string(mapSeed) + "-" + std::to_string(kNeighbor);
        cacheFileName = filename;
        graph.generateRandomGraph(height, width, obstacles, filename, mapSeed, kNeighbor);
    } else if (mapType == "warehouse") {
        filename = timingType + "-warehouse-" + std::to_string(maxX) + "-" + std::to_string(maxY);
        cacheFileName = filename;
        graph.generateWareHouse(deliveryWidth, maxX, maxY, filename, mapSeed);
    } else if (mapType == "hardcoded") {
        filename = "hardcoded";
        graph.generateHardCodedGraph(filename, mapSeed);
        agentNum = 3;
        agentSeed = 1;
    } else if (mapType == "dot") {
        filename = mapFile;
        graph.generateDOTGraph(filename);
    } else if (mapType == "graphml") {
        filename = mapFile;
        if (boost::algorithm::ends_with(filename, ".xml")) { filename = filename.substr(0, filename.length() - 4); }
        cacheFileName = timingType + "-" + mapName;
        graph.generateGraphMLGraph(filename);
        taskFileType = "xml";
    } else if (mapType == "mapf") {
        filename = mapFile;
        if (boost::algorithm::ends_with(filename, ".map")) { filename = filename.substr(0, filename.length() - 4); }
        cacheFileName = timingType + "-mapf-" + mapName;
        graph.generateMAPFBenchmarkGraph(filename, kNeighbor);
        taskFileType = "scen";
    }
    if (useDP) { graph.generateDelayProbability(mapSeed, minDP, maxDP); }
    // only use in individual
    //    graph.calculateAllPairShortestPath(filename, useDP);

    //     exit(0);

    std::vector<Agent> agents;
    if (taskFile.empty()) {
        graph.calculateAllPairShortestPath(filename, useDP);
        if (mapType == "random") {
            agents = graph.generateRandomAgents(agentNum, agentSeed);
        } else if (mapType == "warehouse") {
            agents = graph.generateWarehouseAgents(agentNum, agentSeed, true);
        } else if (mapType == "hardcoded") {
            agents = graph.generateHardCodedAgents(agentNum);
        } else if (mapType == "dot") {
            agents = graph.generateRandomAgents(agentNum, agentSeed);
        } else if (mapType == "graphml") {
            agents = graph.generateRandomAgents(agentNum, agentSeed);
        } else if (mapType == "mapf") {
            agents = graph.generateRandomAgents(agentNum, agentSeed);
        } else {
            assert(0);
        }
    } else {
        std::string taskFilename = taskFile;
        if (taskFileType == "xml") {
            if (boost::algorithm::ends_with(taskFilename, ".xml")) {
                taskFilename = taskFilename.substr(0, taskFilename.length() - 4);
            }
            agents = graph.loadXMLAgents(taskFilename, agentNum, agentSkip);
        } else if (taskFileType == "scen") {
            if (boost::algorithm::ends_with(taskFilename, ".scen")) {
                taskFilename = taskFilename.substr(0, taskFilename.length() - 5);
            }
            //            exit(0);
            agents = graph.loadScenAgents(taskFilename, agentNum, agentSkip);
        } else {
            assert(0);
        }
        // no cache for input task file
        // noCache = true;
    }


    MakeSpanType makeSpanType = MakeSpanType::UNKNOWN;
    if (objective == "maximum") {
        makeSpanType = MakeSpanType::MAXIMUM;
    } else if (objective == "average") {
        makeSpanType = MakeSpanType::AVERAGE;
    } else {
        assert(0);
    }

    std::shared_ptr<Solver> solver;
    if (solverType == "default") {
        solver = std::shared_ptr<Solver>(new CBSSolver(graph, agents, makeSpanType, window));
    } else if (solverType == "eecbs") {
        if (solverBinaryFile.empty()) exit(-1);
        solver = std::shared_ptr<Solver>(new EECBSSolver(graph, agents, makeSpanType, solverBinaryFile, suboptimality));
    } else if (solverType == "lns") {
        if (solverBinaryFile.empty()) exit(-1);
        solver = std::shared_ptr<Solver>(new LNSSolver(graph, agents, makeSpanType, solverBinaryFile));
    } else if (solverType == "ccbs") {
        if (solverBinaryFile.empty()) exit(-1);
        solver = std::shared_ptr<Solver>(new CCBSSolver(graph, agents, makeSpanType, solverBinaryFile, mapType));
    } else if (solverType == "individual") {
        graph.calculateAllPairShortestPath(filename, useDP);
        solver = std::shared_ptr<Solver>(new IndividualAStarSolver(graph, agents, makeSpanType));
    } else {
        assert(0);
    }

    //    CBSSolver solver(graph, agents);
    solver->debug = debug;
    solver->useDP = useDP;
    solver->allConstraint = allConstraint;
    //    solver->timeout = timeout;
    solver->init();
    bool result;
    if (noCache) {
        result = solver->solve();
        if (result) { solver->removeFollowingConflict(solver); }
    } else {
        result = solver->solveWithCache(solver, cacheFileName, agentSeed);
    }

    if (!result) { exit(-1); }

    //    double approx = solver->approxAverageMakeSpan(*solver->solution);
    //    std::shared_ptr<ContinuousOnlineSimulator> onlineSimulator;
    std::shared_ptr<Simulator> simulator;

    unsigned int finished = 0;

    for (unsigned int i = simulationSeed; finished < iteration; i++) {
        if (useDP) { graph.generateDelayProbability(i, minDP, maxDP); }

        if (simulatorType == "default" || simulatorType == "replan") {
            if (timingType == "continuous") {
                simulator = std::make_unique<ContinuousDefaultSimulator>(graph, agents, i);
            } else {
                simulator = std::make_unique<DiscreteDefaultSimulator>(graph, agents, i);
            }
            if (simulatorType == "replan") {
                auto defaultSimulator = std::dynamic_pointer_cast<DefaultSimulator>(simulator);
                defaultSimulator->replanMode = true;
                defaultSimulator->prioritizedReplan = prioritizedReplan;
                defaultSimulator->prioritizedOpt = prioritizedOpt;
                defaultSimulator->replanNonstop = replanNonstop;
                //                defaultSimulator->replanSuboptimality = replanSuboptimality;
                if (solverType == "eecbs") {
                    std::dynamic_pointer_cast<EECBSSolver>(solver)->suboptimality = suboptimality;
                }
                if (i != simulationSeed) {
                    solver->init();
                    if (!solver->solveWithCache(solver, filename, agentSeed)) { exit(-1); }
                }
                if (solverType == "eecbs") {
                    std::dynamic_pointer_cast<EECBSSolver>(solver)->suboptimality = replanSuboptimality;
                }
            }
        } else if (simulatorType == "online" || simulatorType == "snapshot") {
            if (timingType == "continuous") {
                simulator = std::make_unique<ContinuousOnlineSimulator>(graph, agents, i);
                auto continuousOnlineSimulator = std::dynamic_pointer_cast<ContinuousOnlineSimulator>(simulator);
                continuousOnlineSimulator->removeRedundant = removeRedundant;
                continuousOnlineSimulator->deltaTimestep = deltaTimestep;
                continuousOnlineSimulator->useGroup = useGroup;
                continuousOnlineSimulator->topoGraphType = topoGraphType;
                if (simulatorType == "snapshot") {
                    continuousOnlineSimulator->snapshot = true;
                    continuousOnlineSimulator->snapshotOrder = snapshotOrder;
                }
            } else {
                simulator = std::make_unique<DiscreteOnlineSimulator>(graph, agents, i);
                auto discreteOnlineSimulator = std::dynamic_pointer_cast<DiscreteOnlineSimulator>(simulator);
                discreteOnlineSimulator->onlineOpt = onlineOpt;
                discreteOnlineSimulator->groupDetermined = useGroupDetermined;
                discreteOnlineSimulator->topoGraphType = topoGraphType;
            }
            auto onlineSimulator = std::dynamic_pointer_cast<OnlineSimulator>(simulator);
            onlineSimulator->isHeuristicFeasibilityCheck = !naiveFeasibilityCheck;
            onlineSimulator->isHeuristicCycleCheck = !naiveCycleCheck;
            onlineSimulator->isOnlyCycleCheck = onlyCycleCheck;
            onlineSimulator->isFastCycleCheck = fastCycleCheck;
            onlineSimulator->isFeasibilityType = feasibilityType;
        } else if (simulatorType == "pibt") {
            if (timingType == "continuous") {
                simulator = std::make_unique<ContinuousPIBTSimulator>(graph, agents, i);
            } else {
                simulator = std::make_unique<DiscretePIBTSimulator>(graph, agents, i);
            }
        } else if (simulatorType == "btpg") {
            if (timingType == "continuous") {
                assert(0);
            } else {
                simulator = std::make_unique<DiscreteBTPGSimulator>(graph, agents, i);
            }
        } else if (simulatorType == "ses") {
            if (timingType == "continuous") {
                assert(0);
            } else {
                simulator = std::make_unique<DiscreteSESSimulator>(graph, agents, i);
            }
        } else {
            assert(0);
        }
        simulator->delayRatio = delayRatio;
        simulator->delayType = delayType;
        simulator->setSolver(solver);
        simulator->debug = debug;
        simulator->outputFileName = simulatorOutputFileName;
        simulator->verboseOutput = verboseOutput;

        /*        if (mapType == "hardcoded") {
            CBSNodePtr solution = std::make_shared<CBSNode>();
            std::shared_ptr<AgentPlan> a0 = std::make_shared<AgentPlan>();
            std::shared_ptr<AgentPlan> a1 = std::make_shared<AgentPlan>();
            std::shared_ptr<AgentPlan> a2 = std::make_shared<AgentPlan>();
            solution->plans.push_back(a0);
            solution->plans.push_back(a1);
            solution->plans.push_back(a2);
            (*a0).add(6).add(7).add(8).add(9).add(10).add(11).add(12).add(13);
            (*a1).add(0).add(2).add(4).add(4).add(4).add(10).add(9).add(14).add(15).add(16);
            (*a2).add(1).add(3).add(5).add(5).add(5).add(5).add(5).add(12);
            simulator->setSolution(solution);

        } else {

        }*/

        double currentTimestep = 1;

#ifdef DEBUG_CYCLE
        simulator->debug = true;
#endif
        simulator->setAgents(agents);
        currentTimestep = 0;
        auto start = std::chrono::steady_clock::now();
        auto count = simulator->simulate(currentTimestep, currentTimestep + maxTimestep, delayStart, delayInterval);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (count == agentNum) {
            simulator->resultJson["success"] = true;
        } else {
            simulator->resultJson["success"] = false;
            std::cerr << count << " " << agentNum << std::endl;
        }
        //        simulator->resultJson["map_seed"] = mapSeed;
        //        simulator->resultJson["agent_seed"] = agentSeed;
        simulator->resultJson["iteration"] = i;
        simulator->resultJson["total_time"] = elapsed_seconds.count();
        simulator->writeSimulationOutput();
        simulator->outputJson["result"] = simulator->resultJson;
        if (outputFormat == "json") {
            out << std::setw(2) << std::scientific << std::setprecision(12) << simulator->outputJson << std::endl;
        } else if (outputFormat == "bson") {
            std::vector<std::uint8_t> v = nlohmann::json::to_bson(simulator->outputJson);
            out.write(reinterpret_cast<const char *>(v.data()), v.size());
        }
        finished++;
    }


    if (fout.is_open()) { fout.close(); }

    return 0;
}
