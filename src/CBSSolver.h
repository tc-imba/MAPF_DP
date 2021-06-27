//
// Created by liu on 27/4/2021.
//

#ifndef MAPF_DP_CBSSOLVER_H
#define MAPF_DP_CBSSOLVER_H

#include "Solver.h"
#include <vector>
#include <queue>
#include <memory>
#include <limits>

#include <boost/functional/hash.hpp>

class CBSSolver : public Solver {
private:
    constexpr static auto INVALID = std::numeric_limits<unsigned int>::max();

    struct Constraint {
        unsigned int agentId;
        unsigned int nodeId;
        unsigned int state;

        bool operator==(const Constraint &that) const {
            return agentId == that.agentId && nodeId == that.nodeId && state == that.state;
        }
    };

    struct Label {
        unsigned int nodeId;    // id of the node
        unsigned int state;     // state of the agent
        double estimatedTime;   // expected timestep from start
        double heuristic;       // expected timestep to goal
    };

    struct AgentPlan {
        std::deque<Label> path; // planned path is consisted of labels
    };

    struct CBSNode {
        Constraint constraint;
        double key;
        std::vector<std::shared_ptr<AgentPlan >> plans;
        std::shared_ptr<CBSNode> parent;
        unsigned int depth;
    };

    struct FocalNode {
        Label label;
        unsigned int conflicts;
        std::shared_ptr<FocalNode> parent;
        bool visited = false;
    };

    typedef std::shared_ptr<CBSNode> CBSNodePtr;
    typedef std::shared_ptr<FocalNode> FocalNodePtr;

    struct LabelComp {
        bool operator()(Label const &a, Label const &b) const {
            return a.estimatedTime + a.heuristic < b.estimatedTime + b.heuristic;
        }
    };

    struct CBSNodePtrComp {
        bool operator()(CBSNodePtr const &a, CBSNodePtr const &b) const {
            if (a->key == b->key) return a->depth < b->depth;
            return a->key > b->key;
        }
    };

    struct FocalNodePtrComp {
//        LabelComp comp;
        double key;

        explicit FocalNodePtrComp(double key) : key(key) {}

        bool operator()(FocalNodePtr const &a, FocalNodePtr const &b) const {
            double keyA = a->label.estimatedTime + a->label.heuristic;
            double keyB = b->label.estimatedTime + b->label.heuristic;
            if (keyA <= key && keyB <= key) {
                if (a->conflicts == b->conflicts) return keyA > keyB;
                return a->conflicts > b->conflicts;
            }
            if (keyA > key && keyB > key) {
                if (keyA == keyB) return a->conflicts > b->conflicts;
                return keyA > keyB;
            }
            if (keyA > key) return true;
            if (keyB > key) return false;
            return true;
//            return !comp(a->label, b->label);
        }
    };

    struct LabelHash {
        size_t operator()(const Label &a) const {
            return boost::hash<std::pair<unsigned int, unsigned int>>{}({a.nodeId, a.state});
        }
    };

    struct LabelEqual {
        bool operator()(const Label &a, const Label &b) const {
            return a.state == b.state && a.nodeId == b.nodeId;
        }
    };

/*    struct ConstraintHash {
        size_t operator()(const Constraint &a) const {
            return boost::hash<std::tuple<unsigned int, unsigned int, unsigned int>>{}({a.agentId, a.nodeId, a.state});
        }
    };*/

    std::priority_queue<CBSNodePtr, std::vector<CBSNodePtr>, CBSNodePtrComp> queue;
    std::unordered_map<unsigned int, std::map<unsigned int, double> > estimateMap;
    std::unordered_map<unsigned int, unsigned int> goalAgentMap;
    std::unordered_set<Label, LabelHash, LabelEqual> constraintSet;
    std::unordered_map<Label, std::vector<unsigned int>, LabelHash, LabelEqual> conflictMap;
    unsigned int window;
    unsigned int seed;

    std::shared_ptr<AgentPlan> focalSearch(CBSNode &cbsNode, unsigned int agentId, double key);

    std::vector<CBSSolver::Constraint> findConflict(const Label &label, unsigned int agentId, bool find, bool edit);

    std::vector<CBSSolver::Constraint> findConflicts(CBSNode &cbsNode);

    double getEstimate(const Label &label, const Label &parentLabel);

    unsigned int generateConstraintSet(CBSNode &cbsNode);

public:
    CBSNodePtr solution;
    unsigned int currentTimestep;

    CBSSolver(Graph &graph, unsigned int agentNum) : Solver(graph, agentNum) {};

    void init(unsigned int _seed = 0, MakeSpanType makeSpanType = MakeSpanType::MAXIMUM);

    void initCBS(unsigned int _window = 1000);

    bool step();

    bool simulate();

    double approxAverageMakeSpan(CBSNode &cbsNode);
};


#endif //MAPF_DP_CBSSOLVER_H
