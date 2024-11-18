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

extern pros::MotorGroup lb_motors;
extern pros::Rotation lb_rotation_sensor;

extern pros::Optical optical_sensor;
extern pros::Distance mogo_distance;

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

class PID {
    public: float kP;
            float kI;
            float kD;
            float last_delta_position;
            float integral;
            float integral_threshold;
            bool sign;
            bool start;
            int oscillated;
            int dt;

            PID(float kP, float kI, float kD, float integral_threshold, int dt) {
                this->kP = kP;
                this->kI = kI;
                this->kD = kD;
                this->integral_threshold = integral_threshold;
                this->dt = dt;
            }
};

extern Position position;

extern void pure_pursuit();
extern void track_robot();

extern void turn_to_face(double heading, PID pid, double mult = 1);
extern void turn_to_face_new(double heading, PID pid, double mult = 1);
extern void move_one_dir(double pos, PID pid, char axis, double dir = 1, int timeout = 2000);
extern void move_one_dir_advanced(Point target, PID pid, PID turnpid, char axis, double dir = 1, int timeout = 2000, double tolerance = 0.60);
extern void move_turn_to_point(Point target, PID pid, PID turnpid, bool back, char axis, double dir = 1, double turnmult = 1, int timeout = 2000, double tolerance = 0.60);
extern void move_to_point_straight(Point target, PID movepid, PID turnpid, char axis, bool turn, bool xrev = false, double mult = 1, double heading = position.theta, int timeout = 3000, bool invert = true);

#define ROBOT_HPP
#endif
