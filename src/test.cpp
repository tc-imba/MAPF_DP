//
// Created by liuyh on 3/4/2023.
//

#include <Mathematics/DistPointOrientedBox.h>
#include <Mathematics/Vector2.h>
#include <Mathematics/IntrOrientedBox2Circle2.h>
#include <iostream>

int main() {
//    gte::OrientedBox2<double> box;
//    gte::Vector2<double> point = {0, 1.1};
//    gte::DCPQuery<double, gte::Vector2<double>, gte::OrientedBox2<double>> mQuery;
//    auto result = mQuery(point, box);
//    std::cout << result.distance << std::endl;


    gte::Circle2<double> agentCircle({0, 3}, sqrt(2) / 4);
    gte::Vector2<double> agentVelocity = {0, 1};

    gte::OrientedBox2<double> obstacleBox;
    obstacleBox.extent = {0.5, 0.5};
    obstacleBox.center = {0, 4};

    gte::Vector2<double> obstacleVelocity = {0, 0};
    gte::FIQuery<double, gte::OrientedBox2<double>, gte::Circle2<double>> mQuery;

    auto result = mQuery(obstacleBox, obstacleVelocity, agentCircle, agentVelocity);
    std::cout << result.intersectionType << " " << result.contactTime << std::endl;

    return 0;
}
