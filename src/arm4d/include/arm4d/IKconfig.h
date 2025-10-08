#pragma once
#include <cmath>
#include <Eigen/Dense>

namespace IKconfig {
    constexpr double shift_x = 127.5;
    constexpr double shift_y = 127.5;

    constexpr double L0x = 71.024;
    constexpr double L0y = 395.0;
    constexpr double L1 = 300.0;
    constexpr double L2 = 300.0;

    constexpr double L1_2 = L1*L1;
    constexpr double L2_2 = L2*L2;

    const Eigen::Vector3d baseOffset(0.0, 0.0, 26.0);
    const Eigen::Vector3d endEffectorOffset(100.0, 0.0, 4.5);
}

namespace CollisionConfig {
    constexpr double robotRadius = 60.0; // radius of the robot arm for collision detection
    constexpr double manipulatorRadius = 0.5; // radius of the manipulator for collision detection
}