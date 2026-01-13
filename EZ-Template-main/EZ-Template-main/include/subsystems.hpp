#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor bottomRollers(14);
inline pros::Motor topRollers(5);
inline pros::Motor topIntake(-7);

inline pros::MotorGroup leftChassis({-20, -19, 12});
inline pros::MotorGroup rightChassis({15, 16, -17});

inline ez::Piston descore('B');
inline ez::Piston matchLoader('H');
inline ez::Piston stopPiston('F');
inline pros::Distance front(1);
inline pros::Distance side(13);