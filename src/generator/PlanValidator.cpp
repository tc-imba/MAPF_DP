//
// Created by liuyh on 6/2/2023.
//

#include <vector>
#include <set>
#include <map>
#include <fstream>

#include "../utils/ezOptionParser.hpp"


int main(int argc, const char *argv[]) {

    ez::ezOptionParser optionParser;

    optionParser.overview = "Multi Agent Path Finding Plan Validator";
    optionParser.syntax = "./MAPF_Plan_Validator [OPTIONS]";
    optionParser.footer = "";

//    optionParser.add("maps/barcamap-10-0-default.cbs", false, 1, 0, "Plan File", "--plan");
    optionParser.add("maps/hardcode.cbs", false, 1, 0, "Plan File", "--plan");
//    optionParser.add("maps/random-30-30-90-0", false, 1, 0, "Map File", "--map");

    auto validAgents = new ez::ezOptionValidator("u4", "ge", "1");
//    optionParser.add("4", false, 1, 0, "Agents Number", "-a", "--agents", validAgents);

    std::string planName;
    optionParser.parse(argc, argv);
    optionParser.get("--plan")->getString(planName);
//    optionParser.get("--agents")->getULong(agents);

    unsigned long agents = 0;
    std::vector<std::vector<unsigned int>> paths;

    std::ifstream planFile(planName);
    if (!planFile.is_open()) {
        exit(-1);
    }
    while (true) {
        std::string temp;
        while (std::getline(planFile, temp)) {
            if (!temp.empty()) break;
        }
        if (temp.empty()) break;
        auto length = std::stoul(temp);
        paths.emplace_back();
        for (unsigned int j = 0; j < length; j++) {
            std::getline(planFile, temp);
            auto nodeId = std::stoul(temp);
            paths[agents].emplace_back(nodeId);
        }
        agents++;
    }
    planFile.close();

    std::map<unsigned int, unsigned int> last;
    unsigned int followingConflicts = 0, cycleConflicts = 0;

    for (unsigned int state = 0;; state++) {
        int count = 0;
        std::map<unsigned int, unsigned int> following;
        for (unsigned int i = 0; i < agents; i++) {
            if (state < paths[i].size()) {
                auto it = last.find(paths[i][state]);
                if (it != last.end() && i != it->second) {
                    following.emplace(i, it->second);
                }
            } else {
                count++;
            }
        }
        for (unsigned int i = 0; i < agents; i++) {
            if (state < paths[i].size()) {
                if (state > 0) {
                    last.erase(paths[i][state - 1]);
                }
                last[paths[i][state]] = i;
            }
        }

        followingConflicts += following.size();
        std::set<unsigned int> cycle;

        while (!following.empty()) {
            auto it = following.begin();
            auto src = it->first;
            auto current = it->second;
            following.erase(it);
            while (true) {
                it = following.find(current);
                if (it != following.end()) {
                    current = it->second;
                    following.erase(it);
                    if (src == current) {
                        cycleConflicts += 1;
                    }
                } else {
                    break;
                }
            }
        }

        if (count == agents) {
            break;
        }
    }

    std::cout << agents << "," << followingConflicts << "," << cycleConflicts << std::endl;

    return 0;
}
