#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#ifndef ROBOT_HPP

extern pros::Controller controller;

extern pros::MotorGroup left_drive_motors;
extern pros::MotorGroup right_drive_motors;

extern pros::adi::DigitalOut mogo_piston;

extern pros::Rotation left_encoder;
extern pros::Rotation back_encoder;
extern pros::Imu inertial_sensor;

#define ROBOT_HPP
#endif
