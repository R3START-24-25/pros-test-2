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
extern pros::adi::DigitalOut doinker_piston;

extern pros::Imu inertial_sensor;
extern pros::Rotation left_encoder;
extern pros::Rotation back_encoder;

extern pros::Motor intake_motor;

//extern pros::Optical optical_sensor;
//extern pros::Distance distance_sensor;

class Position {
    public: float x = 0;
            float y = 0;
            float theta = 0;
};

class Point {
    public: double x, y;
            Point(double X, double Y) {
                x = X; y = Y;
            }
            Point();
};

extern Position position;

extern void pure_pursuit();
//extern void move_to_point_straight(Point target, double kP_linear, double kP_turn);

#define ROBOT_HPP
#endif
