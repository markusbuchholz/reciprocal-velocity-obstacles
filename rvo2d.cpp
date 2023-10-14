//Markus Buchholz, 2023
//g++ rvo2d.cpp -o t -I/usr/include/python3.10 -lpython3.10

#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include "matplotlibcpp.h"

const int STEPS = 100;

namespace plt = matplotlibcpp;



//-------------------------------------------------------------------------
class Robot {
private:
    std::array<double, 2> position;
    std::array<double, 2> goal;
    double speed;
    double radius;
    std::vector<std::array<double, 2>> path;

public:
    Robot(std::array<double, 2> position, std::array<double, 2> goal, double speed, double radius)
        : position(position), goal(goal), speed(speed), radius(radius) {
        path.push_back(position);
    }

    void compute_new_velocity(const Robot& other) {
        std::array<double, 2> rel_position = {
            other.position[0] - position[0],
            other.position[1] - position[1]
        };

        double distance = std::sqrt(rel_position[0]*rel_position[0] + rel_position[1]*rel_position[1]);

        if (distance < radius + other.radius) {
            std::array<double, 2> avoidance = {
                -rel_position[1],
                rel_position[0]
            };
            double norm = std::sqrt(avoidance[0]*avoidance[0] + avoidance[1]*avoidance[1]);
            position[0] += (avoidance[0] / norm) * speed;
            position[1] += (avoidance[1] / norm) * speed;
        } else {
            std::array<double, 2> direction = {
                goal[0] - position[0],
                goal[1] - position[1]
            };
            double norm = std::sqrt(direction[0]*direction[0] + direction[1]*direction[1]);
            position[0] += (direction[0] / norm) * speed;
            position[1] += (direction[1] / norm) * speed;
        }

        path.push_back(position);
    }

    const std::vector<std::array<double, 2>>& getPath() const {
        return path;
    }
};

//-------------------------------------------------------------------------


void plot2D(std::vector<float> r1_x, std::vector<float> r1_y, std::vector<float> r2_x, std::vector<float> r2_y)
{

    plt::named_plot("robot 1", r1_x, r1_y);
    plt::named_plot("robot 2", r2_x, r2_y);
    plt::title("Reciprocal Velocity Obstacles (RVO)");
    plt::xlabel("x");
    plt::ylabel("y");
    plt::show();
}

//-------------------------------------------------------------------------

int main() {
    Robot robot1({0, 0}, {100, 100}, 2, 15);
    Robot robot2({100, 100}, {0, 0}, 2, 15);

    for (int i = 0; i < STEPS; i++) {
        robot1.compute_new_velocity(robot2);
        robot2.compute_new_velocity(robot1);
    }

    const auto& path1 = robot1.getPath();
    const auto& path2 = robot2.getPath();

    std::vector<float> x1, y1, x2, y2;
    for (const auto& p : path1) {
        x1.push_back(p[0]);
        y1.push_back(p[1]);
    }
    for (const auto& p : path2) {
        x2.push_back(p[0]);
        y2.push_back(p[1]);
    }

    plot2D(x1, y1, x2, y2);

    return 0;
}
