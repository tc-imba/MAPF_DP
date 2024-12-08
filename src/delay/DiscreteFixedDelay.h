//
// Created by liuyh on 25/11/2024.
//

#ifndef MAPF_DP_DISCRETEFIXEDDELAY_H
#define MAPF_DP_DISCRETEFIXEDDELAY_H

#include <set>

class DiscreteFixedDelay {
protected:
    std::set<unsigned int> delayedSet;
    unsigned int delaySetSize;
    unsigned int seed;
    const double &delayRatio;

public:
    DiscreteFixedDelay(unsigned int delaySetSize, unsigned int seed, const double &delayRatio)
        : delaySetSize(delaySetSize), seed(seed), delayRatio(delayRatio) {};

    void update(unsigned int currentTimestep, unsigned int delayStart, unsigned int delayInterval);

    void clear();

    [[nodiscard]] const auto& getDelayedSet() const { return delayedSet; }
};


#endif//MAPF_DP_DISCRETEFIXEDDELAY_H
