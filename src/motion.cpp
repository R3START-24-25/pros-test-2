#include "pros/rtos.hpp"
#include <cmath>
#include "robot.hpp"

double disp_between_pts(Point point_one, Point point_two) {
    double dx = point_two.x - point_one.x;
    double dy = point_two.y - point_one.y;

    double dist = sqrt(dx*dx + dy*dy);
    dist *= atan2(point_two.y - point_one.y, point_two.x - point_one.x) < 0 ? -1 : 1;

    return dist;
}

double move_pid(PID pid, Point target, char axis, bool rev = false) {
    if (pid.start) {
        pid.last_delta_position = 0;
        pid.oscillated = 0;
        pid.integral = 0;
        pid.start = false;
    }

    Point current_position = Point(position.x, position.y);
    double delta_position = disp_between_pts(current_position, target);
    if (rev) delta_position = disp_between_pts(target, current_position);

    if (fabs(delta_position) < 0.2) return 0;

    bool localsign = delta_position > 0 ? true : false;
    if (!pid.start && pid.sign != localsign) pid.oscillated++;
    if (pid.oscillated == 1) return 0;
    pid.sign = localsign;

    pid.integral = fabs(delta_position) > 2.5 && fabs(delta_position) < pid.integral_threshold
        ? pid.integral + delta_position
        : 0;
    
    float derivative = delta_position - pid.last_delta_position;
    if (rev) derivative = pid.last_delta_position - delta_position;
    pid.last_delta_position = delta_position;
    if (pid.start) {
        derivative = 0;
        pid.start = false;
    }

    float p_i_d = (delta_position * pid.kP) + (pid.integral * pid.kI) + (derivative * pid.kD);
    return p_i_d;
}

double move_pid_one_dir(PID pid, double target, char axis) {
    if (pid.start) {
        pid.last_delta_position = 0;
        pid.oscillated = 0;
        pid.integral = 0;
        pid.start = false;
    }

    double current_position = axis == 'y' ? position.y : position.x;
    double delta_position = target - current_position;

    bool localsign = delta_position > 0 ? true : false;
    if (!pid.start && pid.sign != localsign) pid.oscillated++;
    pid.sign = localsign;

    pid.integral = fabs(delta_position) > 2.5 && delta_position < pid.integral_threshold
        ? pid.integral + delta_position
        : 0;
    
    float derivative = delta_position - pid.last_delta_position;
    pid.last_delta_position = delta_position;
    if (pid.start) {
        derivative = 0;
        pid.start = false;
    }

    float p_i_d = (delta_position * pid.kP) + (pid.integral * pid.kI) + (derivative * pid.kD);
    return p_i_d;
}

double turn_pid(PID pid, double target) {
    if (pid.start) {
        pid.last_delta_position = 0;
        pid.oscillated = 0;
        pid.integral = 0;
    }

    double delta_position = position.theta - target;

    bool localsign = delta_position > 0 ? true : false;
    if (!pid.start && pid.sign != localsign) pid.oscillated++;
    pid.sign = localsign;

    if (pid.oscillated > 1) return 0;

    if (fabs(delta_position) > 180) delta_position = (delta_position < 0 ? 1 : -1) * (360 - fabs(delta_position));

    pid.integral = fabs(delta_position) > 1.5
        && delta_position < pid.integral_threshold
        ? pid.integral + delta_position
        : 0;
    
    float derivative = delta_position - pid.last_delta_position;
    pid.last_delta_position = delta_position;
    if (pid.start) {
        derivative = 0;
        pid.start = false;
    }

    float p_i_d = (delta_position * pid.kP) + (pid.integral * pid.kI) + (derivative * pid.kD);
    return p_i_d;
}

void turn_to_face(double heading, PID pid, double mult) {
    int time = 0;
    double turn_pid_out = 127;
    pid.start = true;

    while (fabs(turn_pid_out) > 0.6) {
        turn_pid_out = turn_pid(pid, heading);
        if (fabs(turn_pid_out) <= 0.6) break;

        turn_pid_out = fabs(turn_pid_out) < 18 ? (turn_pid_out > 0 ? 18 : -18) : turn_pid_out;
        turn_pid_out = fabs(turn_pid_out) > 60 ? (turn_pid_out > 0 ? 60 : -60) : turn_pid_out;

        left_drive_motors.move(turn_pid_out*mult);
        right_drive_motors.move(turn_pid_out*mult);

        pros::delay(5);
        
        time += 5;
        if (time >= 2500) break;
    }

    left_drive_motors.move(0); right_drive_motors.move(0);
}

void turn_to_face_new(double heading, PID pid, double mult) {
    int time = 0;
    double turn_pid_out = 127;
    pid.start = true;

    while (fabs(turn_pid_out) > 0.6) {
        turn_pid_out = turn_pid(pid, heading);
        if (fabs(turn_pid_out) <= 0.6) break;

        turn_pid_out = fabs(turn_pid_out) < 16 ? (turn_pid_out > 0 ? 16 : -16) : turn_pid_out;
        turn_pid_out = fabs(turn_pid_out) > 70 ? (turn_pid_out > 0 ? 70 : -70) : turn_pid_out;

        left_drive_motors.move(turn_pid_out*mult);
        right_drive_motors.move(turn_pid_out*mult);

        pros::delay(5);
        
        time += 5;
        if (time >= 2500) break;
    }

    left_drive_motors.move(0); right_drive_motors.move(0);
}

void move_one_dir(double pos, PID pid, char axis, double dir, int timeout) {
    int time = 0;
    double linear_pid_out = 127;
    pid.start = true;

    while (fabs(linear_pid_out) > 1.5) {
        linear_pid_out = move_pid_one_dir(pid, pos, axis);

        linear_pid_out = fabs(linear_pid_out) < 12 && fabs(linear_pid_out) > 0.75 ? (linear_pid_out > 0 ? 12 : -12) : linear_pid_out;
        linear_pid_out = fabs(linear_pid_out) > 60 ? (linear_pid_out > 0 ? 60 : -60) : linear_pid_out;

        left_drive_motors.move(linear_pid_out * dir);
        right_drive_motors.move(-linear_pid_out * dir);

        pros::delay(5);
        time += 5;

        if (time >= timeout) break;
    }

    left_drive_motors.move(0); right_drive_motors.move(0);
}

void move_one_dir_advanced(Point target, PID pid, PID turnpid, char axis, double dir, int timeout, double tolerance) {
    int time = 0;
    double linear_pid_out = 127;
    pid.start = true;
    double pos = axis == 'x' ? target.x : target.y;

    double abs_turn_to = atan2f(target.x - position.x, target.y - position.y) * (180/M_PI);
    if (abs_turn_to < 0) abs_turn_to += 180;

    //turn_to_face_new(abs_turn_to, turnpid);
    //std::cout << "abs turn to: " << abs_turn_to << std::endl;

    while (fabs(linear_pid_out) > tolerance) {
        linear_pid_out = move_pid_one_dir(pid, pos, axis);

        linear_pid_out = fabs(linear_pid_out) < 14 && fabs(linear_pid_out) > tolerance ? (linear_pid_out > 0 ? 14 : -14) : linear_pid_out;
        linear_pid_out = fabs(linear_pid_out) > 90 ? (linear_pid_out > 0 ? 90 : -90) : linear_pid_out;

        left_drive_motors.move(linear_pid_out * dir);
        right_drive_motors.move(-linear_pid_out * dir);

        pros::delay(5);
        time += 5;

        if (time >= timeout) break;
    }

    left_drive_motors.move(0); right_drive_motors.move(0);
}

void move_turn_to_point(Point target, PID pid, PID turnpid, bool back, char axis, double dir, double turnmult, int timeout, double tolerance) {
    double dx = target.x - position.x;
    double dy = target.y - position.y;
    double abs_turn_to = atan2f(target.x - position.x, target.y - position.y) * (180.0/M_PI);
    std::cout << "abs: " << abs_turn_to;
    if (!back) abs_turn_to += 180;
    while (abs_turn_to < 0) abs_turn_to += 360;
    while (abs_turn_to > 360) abs_turn_to -= 360;

    turn_to_face_new(abs_turn_to, turnpid, turnmult);
    pros::delay(500);
    std::cout << "t: " << position.theta;
    move_one_dir_advanced(target, pid, turnpid, axis, dir, timeout, tolerance);
}

void move_to_point_straight(Point target, PID movepid, PID turnpid, char axis, bool turn, bool xrev, double mult, double heading, int timeout, bool invert) {
    int time = 0;
    double abs_turn_to = atan2f(target.x - position.x, target.y - position.y) * (180/M_PI);
    if (abs_turn_to < 0) abs_turn_to += 360;

    double turn_pid_out = 127;
    double linear_pid_out = 127;

    turnpid.start = true;
    movepid.start = true;
    while (fabs(turn_pid_out) > 3.6 || fabs(linear_pid_out) > 0.5) {
        double dx = target.x - position.x; //30
        double dy = target.y - position.y; //-5
        if (fabs(dx) + fabs(dy) > 3) {
            abs_turn_to = atan2f(dx, dy) * (180/M_PI); //100
            if (invert) abs_turn_to += 180;
            while (abs_turn_to < 0) abs_turn_to += 360;
            std::cout << "abs_turn_to: " << abs_turn_to << std::endl;
        }

        turn_pid_out = turn_pid(turnpid, abs_turn_to); //100

        //turn_pid_out = fabs(turn_pid_out) < 12 && fabs(turn_pid_out) > 0.6 ? (turn_pid_out > 0 ? 12 : -12) : turn_pid_out;
        turn_pid_out = fabs(turn_pid_out) > 60 ? (turn_pid_out > 0 ? 60 : -60) : turn_pid_out;


        linear_pid_out = move_pid(movepid, target, axis, xrev);

        linear_pid_out = fabs(linear_pid_out) < 10 && fabs(linear_pid_out) > 0.5 ? (linear_pid_out > 0 ? 10 : -10) : linear_pid_out;
        linear_pid_out = fabs(linear_pid_out) > 60 ? (linear_pid_out > 0 ? 60 : -60) : linear_pid_out;

        if (xrev) linear_pid_out *= 1;
        
        left_drive_motors.move((turn_pid_out + linear_pid_out)*mult);
        right_drive_motors.move((turn_pid_out - linear_pid_out)*mult);

        pros::delay(5);
        time += 5;

        if (time >= timeout) break;
    }

    left_drive_motors.move(0); right_drive_motors.move(0);

    if (turn) turn_to_face(heading, turnpid);
}
