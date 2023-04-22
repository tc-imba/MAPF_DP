//
// Created by liuyh on 10/4/2023.
//

#include <boost/filesystem.hpp>
#include <boost/algorithm/string_regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/process.hpp>
//#include <fstream>

#include "../../Continuous-CBS/tinyxml2.h"
#include "CCBSSolver.h"

bool CCBSSolver::readSolution(const std::string &filename) {
    auto node = std::make_shared<CBSNode>();
    node->constraint = {INVALID, INVALID, INVALID};
    node->plans.resize(agents.size());
    node->parent = nullptr;
    node->depth = 0;

    tinyxml2::XMLDocument doc;
    doc.LoadFile(filename.c_str());

    auto log = doc.FirstChildElement("root")->FirstChildElement("log");
    auto summary = log->FirstChildElement();
    executionTime = std::strtod(summary->Attribute("time"), nullptr);

//    std::cout << executionTime << std::endl;

    auto agentElem = summary;
    for (unsigned int i = 0; i < agents.size(); i++) {
        agentElem = agentElem->NextSiblingElement();
        auto agentId = std::strtoul(agentElem->Attribute("number"), nullptr, 10);
        if (agentId != i) return false;
        auto &plan = node->plans[agentId] = std::make_shared<AgentPlan>();
        auto pathElem = agentElem->FirstChildElement();
        auto sectionElem = pathElem->FirstChildElement();
        auto totalTime = std::strtod(pathElem->Attribute("duration"), nullptr);
//        std::cout << i << " " << agentId << std::endl;
        while (sectionElem) {
            auto labelId = std::strtoul(sectionElem->Attribute("number"), nullptr, 10);
            auto startX = std::strtoul(sectionElem->Attribute("start_i"), nullptr, 10);
            auto startY = std::strtoul(sectionElem->Attribute("start_j"), nullptr, 10);
            auto goalX = std::strtoul(sectionElem->Attribute("goal_i"), nullptr, 10);
            auto goalY = std::strtoul(sectionElem->Attribute("goal_j"), nullptr, 10);
            auto time = std::strtod(sectionElem->Attribute("duration"), nullptr);
            auto startNode = graph.getNodeIdByGridPos(startX, startY);
            auto goalNode = graph.getNodeIdByGridPos(goalX, goalY);
            if (startNode == std::numeric_limits<unsigned int>::max()) return false;
            if (goalNode == std::numeric_limits<unsigned int>::max()) return false;
//            std::cout << labelId << " " << startNode << " " << goalNode << std::endl;
            if (plan->path.empty()) {
                plan->add(startNode);
                plan->path.back().heuristic = totalTime;
            } else if (plan->path.back().nodeId != startNode) return false;
            if (labelId + 1 != plan->path.size()) return false;
            auto timeBefore = plan->path.back().estimatedTime;
            plan->add(goalNode);
            plan->path.back().estimatedTime = timeBefore + time;
            plan->path.back().heuristic = totalTime - plan->path.back().estimatedTime;
            sectionElem = sectionElem->NextSiblingElement();
        }
        // empty path
        if (plan->path.empty()) {
            plan->add(agents[i].goal);
        }
    }

    solution = node;
    return true;

}


bool CCBSSolver::solve() {
    using namespace boost::filesystem;
    using namespace boost::process;

    executionTime = -1;

    path ph = path("ccbs_results") / unique_path();
    create_directories(ph);
    path taskFile = ph / "task";
    std::string taskFileName = taskFile.string() + ".xml";
    std::string configFileName = (ph / "config.xml").string();
    std::string srcMapFileName = (current_path() / (graph.getFileName() + ".xml")).string();
    std::string mapFileName = (ph / "map.xml").string();
    std::string outputFileName = (ph / "task_log.xml").string();

//    std::cout << taskFileName << std::endl;
//    std::cout << configFileName << std::endl;
//    std::cout << srcMapFileName << std::endl;
//    std::cout << mapFileName << std::endl;
//    std::cout << outputFileName << std::endl;

    copy_file(srcMapFileName, mapFileName);
    graph.saveAgentsXML(taskFile.string(), agents);
    saveConfig(configFileName);

    std::vector<std::string> arguments;
    arguments.emplace_back(solverBinaryFile);
    arguments.emplace_back("map.xml");
    arguments.emplace_back("task.xml");
    arguments.emplace_back("config.xml");

    std::string argumentsStr = boost::algorithm::join(arguments, " ");
//    std::cout << argumentsStr << std::endl;

    child c(argumentsStr, std_out > null, std_err > null, start_dir(ph));
//    auto start = std::chrono::steady_clock::now();
    c.wait();

//    auto end = std::chrono::steady_clock::now();
//    std::chrono::duration<double> elapsed_seconds = end - start;
//    executionTime = elapsed_seconds.count();

    success = readSolution(outputFileName);

    if (success) {
        remove_all(ph);
    }
    prioritizedReplan = false;
    return success;
}

void CCBSSolver::saveConfig(const std::string &filename) {
    tinyxml2::XMLDocument doc;

    doc.InsertEndChild(doc.NewDeclaration());
    auto root = doc.NewElement("root");
    doc.InsertEndChild(root);
    auto algorithm = doc.NewElement("algorithm");
    root->InsertEndChild(algorithm);

    std::vector<std::pair<std::string, std::string>> data = {
            {"use_cardinal",           "true"},
            {"use_disjoint_splitting", "true"},
            {"hlh_type",               "2"},
            {"connectedness",          std::to_string(graph.getKNeighbor())},
            {"focal_weight",           "1.0"},
            {"agent_size",             "0.353553"},
            {"timelimit",              "30"},
            {"precision",              "0.0000001"},
    };

    for (auto &[key, value]: data) {
        auto elem = doc.NewElement(key.c_str());
        elem->SetText(value.c_str());
        algorithm->InsertEndChild(elem);
    }

    doc.SaveFile(filename.c_str());
}
