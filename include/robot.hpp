#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

#ifndef ROBOT_HPP

extern pros::Controller controller;

extern pros::MotorGroup left_drive_motors;
extern pros::MotorGroup right_drive_motors;

extern pros::adi::DigitalOut mogo_piston;

extern pros::Imu inertial_sensor;

#define ROBOT_HPP
#endif
