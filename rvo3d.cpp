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
    std::array<double, 3> position;
    std::array<double, 3> goal;
    double speed;
    double radius;
    std::vector<std::array<double, 3>> path;

public:
    Robot(std::array<double, 3> position, std::array<double, 3> goal, double speed, double radius)
        : position(position), goal(goal), speed(speed), radius(radius) {
        path.push_back(position);
    }

    void compute_new_velocity(const Robot& other) {
        std::array<double, 3> rel_position = {
            other.position[0] - position[0],
            other.position[1] - position[1],
            other.position[2] - position[2]
        };

        double distance = std::sqrt(rel_position[0]*rel_position[0] + rel_position[1]*rel_position[1] + rel_position[2]*rel_position[2]);

        if (distance < radius + other.radius) {
            std::array<double, 3> avoidance = {
                -rel_position[1],
                rel_position[0],
                0
            };
            double norm = std::sqrt(avoidance[0]*avoidance[0] + avoidance[1]*avoidance[1] + avoidance[2]*avoidance[2]);
            position[0] += (avoidance[0] / norm) * speed;
            position[1] += (avoidance[1] / norm) * speed;
            position[2] += (avoidance[2] / norm) * speed;
        } else {
            std::array<double, 3> direction = {
                goal[0] - position[0],
                goal[1] - position[1],
                goal[2] - position[2]
            };
            double norm = std::sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);
            position[0] += (direction[0] / norm) * speed;
            position[1] += (direction[1] / norm) * speed;
            position[2] += (direction[2] / norm) * speed;
        }

        path.push_back(position);
    }

    const std::vector<std::array<double, 3>>& getPath() const {
        return path;
    }
};

//-------------------------------------------------------------------------

void plot3D(std::vector<float> r1_x, std::vector<float> r1_y, std::vector<float> r1_z, std::vector<float> r2_x, std::vector<float> r2_y, std::vector<float> r2_z)
{

    plt::plot3(r1_x, r1_y, r1_z);
    plt::plot3(r2_x, r2_y, r2_z);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("z");
    plt::show();
}

//-------------------------------------------------------------------------

int main() {
    Robot robot1({0, 0, 0}, {100, 100, 100}, 2, 15);
    Robot robot2({100, 100, 100}, {0, 0, 0}, 2, 15);

    for (int i = 0; i < STEPS; i++) {
        robot1.compute_new_velocity(robot2);
        robot2.compute_new_velocity(robot1);
    }

    const auto& path1 = robot1.getPath();
    const auto& path2 = robot2.getPath();

    std::vector<float> x1, y1, z1, x2, y2, z2;
    for (const auto& p : path1) {
        x1.push_back(p[0]);
        y1.push_back(p[1]);
        z1.push_back(p[2]);
    }
    for (const auto& p : path2) {
        x2.push_back(p[0]);
        y2.push_back(p[1]);
        z2.push_back(p[2]);
    }

    plot3D(x1, y1, z1, x2, y2, z2);

    return 0;
}
