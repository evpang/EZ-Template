#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "motoritf.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// Intake constructor
inline pros::Motor intake(-6, pros::MotorGears::blue);
inline pros::Motor arm(-18, pros::MotorGears::red);
inline pros::Rotation armRotation(4);
inline pros::Optical optical(1);

//pneumatics
inline pros::adi::DigitalOut mogo ('H');
inline pros::adi::DigitalOut leftDoinker ('A');
inline pros::adi::DigitalOut rightDoinker ('G');

// arm
void setArmTarget(int targetInCentidegrees);

// alliance
void setAllianceBlue();
void setAllianceRed();
