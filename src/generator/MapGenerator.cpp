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

    optionParser.add("maps", false, 1, 0, "Map Directory", "-d", "--map-directory");

    auto validHeight =  new ez::ezOptionValidator("u4", "gt", "0");
    optionParser.add("30", false, 1, 0, "Height", "--height", validHeight);

    auto validWidth =  new ez::ezOptionValidator("u4", "gt", "0");
    optionParser.add("30", false, 1, 0, "Width", "--width", validWidth);

    auto validObstacles = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("90", false, 1, 0, "Obstacles", "--obstacles", validObstacles);

    auto validMapSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Map Seed", "--map-seed", validMapSeed);

    unsigned long height, width, obstacles, mapSeed;
    std::string mapDirectory;

    optionParser.parse(argc, argv);

    optionParser.get("--map-directory")->getString(mapDirectory);
    optionParser.get("--height")->getULong(height);
    optionParser.get("--width")->getULong(width);
    optionParser.get("--obstacles")->getULong(obstacles);
    optionParser.get("--map-seed")->getULong(mapSeed);

    boost::filesystem::path mapDirectoryPath(mapDirectory);
    if (!boost::filesystem::exists(mapDirectoryPath)) {
        boost::filesystem::create_directory(mapDirectoryPath);
    }

    Graph graph;
    std::string filename = "random-" + std::to_string(height) + "-" + std::to_string(width) + "-" + std::to_string(obstacles) +
               "-" + std::to_string(mapSeed);
    std::string fullFilename = (mapDirectoryPath / filename).string();
    std::cout << fullFilename << std::endl;

    graph.generateRandomGraph(height, width, obstacles, fullFilename, mapSeed);
//    graph.calculateAllPairShortestPath(fullFilename, false);

    return 0;
}