//
// Created by liu on 16/2/2025.
//

#ifndef SIPPSOLVER_H
#define SIPPSOLVER_H

#include <boost/heap/pairing_heap.hpp>

#include "Solver.h"
#include "IndividualSolver.h"

class SIPPSolver : IndividualSolver {
public:
    struct Node;

    typedef boost::heap::pairing_heap<
        std::shared_ptr<Node>,
        boost::heap::compare<IndividualSolver::Node::CompareOpen>
    > heap_open_t;
    typedef boost::heap::pairing_heap<
        std::shared_ptr<Node>,
        boost::heap::compare<IndividualSolver::Node::CompareFocal>
    > heap_focal_t;

    struct Node : IndividualSolver::Node {
        // the handles allow to quickly update a node in the heap
        heap_open_t::handle_type openHandle;
        heap_focal_t::handle_type focalHandle;

        int highGeneration; // the upper bound with respect to generation
        int highExpansion; // the upper bound with respect to expansion
        bool collisionV;

        struct Hash {
            std::size_t operator()(const std::shared_ptr<Node> &n) const {
                size_t seed = 0;
                boost::hash_combine(seed, n->physicalNodeId);
                boost::hash_combine(seed, n->highGeneration);
                return seed;
            }
        };

        struct Pred {
            bool operator()(const std::shared_ptr<Node> &n1, const std::shared_ptr<Node> &n2) const {
                return n1.get() == n2.get() || (
                           n1 && n2 &&
                           n1->physicalNodeId == n2->physicalNodeId &&
                           n1->waitAtGoal == n2->waitAtGoal &&
                           n1->isGoal == n2->isGoal &&
                           n1->highGeneration == n2->highGeneration
                       );
            }
        };
    };

    typedef boost::unordered_map<
        std::shared_ptr<Node>,
        std::list<std::shared_ptr<Node> >,
        Node::Hash,
        Node::Pred
    > hashtable_t;

    std::shared_ptr<Agent> agent;
    AgentPlan plan;

protected:
    heap_open_t openList;
    heap_focal_t focalList;
    hashtable_t allNodesTable;
    std::list<std::shared_ptr<Node> > uselessNodes;

public:
    SIPPSolver(std::shared_ptr<Agent> agent) : agent(std::move(agent)) {
    }

    std::string getSolverName() override { return "sipp"; }

    void solve();

protected:
    void init();

};


#endif//SIPPSOLVER_H
