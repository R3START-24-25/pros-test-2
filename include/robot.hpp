#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

#ifndef ROBOT_HPP

extern pros::Controller controller;

extern pros::MotorGroup left_drive_motors;
extern pros::MotorGroup right_drive_motors;

extern pros::adi::DigitalOut mogo_piston;
extern pros::adi::DigitalOut intake_lift_piston;
extern pros::adi::DigitalOut claw_lift_piston;

extern pros::Imu inertial_sensor;
extern pros::Rotation left_encoder;
extern pros::Rotation back_encoder;

extern pros::MotorGroup intake_motor;

extern pros::Optical optical_sensor;
extern pros::Distance distance_sensor;

class Position {
    public: float x = 0;
            float y = 0;
            float theta = 0;
};

#define ROBOT_HPP
#endif
