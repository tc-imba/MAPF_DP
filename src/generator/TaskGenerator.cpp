//
// Created by liu on 11/12/2022.
//

#include <iostream>
#include <chrono>
#include <boost/filesystem.hpp>
//#include <boost/interprocess/sync/named_mutex.hpp>
#include "../Graph.h"
#include "../utils/ezOptionParser.hpp"

int main(int argc, const char *argv[]) {
    ez::ezOptionParser optionParser;

    optionParser.overview = "Multi Agent Path Finding Map Generator";
    optionParser.syntax = "./MAPF_Map_Generator [OPTIONS]";
    optionParser.footer = "";

    optionParser.add("tasks", false, 1, 0, "Task Directory", "-d", "--task-directory");
    optionParser.add("maps/random-30-30-90-0", false, 1, 0, "Map File", "--map");

    auto validAgents = new ez::ezOptionValidator("u4", "ge", "1");
    optionParser.add("10", false, 1, 0, "Agents Number", "-a", "--agents", validAgents);

    auto validAgentSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Agent Seed", "--agent-seed", validAgentSeed);


    unsigned long agents, agentSeed;
    std::string mapName, taskDirectory;

    optionParser.parse(argc, argv);

    optionParser.get("--map")->getString(mapName);
    optionParser.get("--task-directory")->getString(taskDirectory);
    optionParser.get("--agents")->getULong(agents);
    optionParser.get("--agent-seed")->getULong(agentSeed);

    boost::filesystem::path taskDirectoryPath(taskDirectory);
    if (!boost::filesystem::exists(taskDirectoryPath)) {
        boost::filesystem::create_directory(taskDirectoryPath);
    }

    boost::filesystem::path mapFilePath(mapName);

    Graph graph;
    graph.generateFileGraph(mapName);
    graph.calculateAllPairShortestPath("", false);
    auto generatedAgents = graph.generateRandomAgents(agents, agentSeed);

    std::string mapFileName = boost::filesystem::path(mapName).filename().string();
    std::string filename = mapFileName + "-" + std::to_string(agents) + "-" + std::to_string(agentSeed);
    std::string fullFilename = (taskDirectoryPath / filename).string();
    std::cout << fullFilename << std::endl;

    graph.saveAgents(mapFileName, fullFilename, generatedAgents);

    return 0;
}