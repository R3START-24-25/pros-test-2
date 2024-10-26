#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "robot.hpp"

class Drive {
    const double EulerConstant = std::exp(1.0);
    const float exponential_a = pow(10, (log10(127)/80));
    const float exponential_b(float x) {
        return ((1/(1+pow(EulerConstant, (-1*(x-83.5)/12))))*110)+20;
    }

    public: void movement() {
        int dir = controller.get_analog(ANALOG_LEFT_Y);
        int turn = controller.get_analog(ANALOG_RIGHT_X);

        if (turn <= 80) turn *= exponential_a;
        else if (turn > 0) turn = 127;
        else if (turn >= -80) turn = abs(turn) * exponential_a * -1;
        else if (turn < 0) turn = -127;

        if (dir > 2) dir = exponential_b(dir);
        else if (dir < -2) dir = -1*exponential_b(abs(dir));
        else dir = 0;

        left_drive_motors.move(-turn - dir);
        right_drive_motors.move(-turn + dir);
    }
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

double disp_between_pts(Point point_one, Point point_two) {
    double dx = point_two.x - point_one.x;
    double dy = point_two.y - point_one.y;

    double dist = sqrt(dx*dx + dy*dy);
    dist *= atan2(point_two.y - point_one.y, point_two.x - point_one.x) < 0 ? -1 : 1;

    return dist;
}

double move_pid(PID pid, Point target, char axis) {
    if (pid.start) {
        pid.last_delta_position = 0;
        pid.oscillated = 0;
        pid.integral = 0;
        pid.start = false;
    }

    Point current_position = Point(position.x, position.y);
    double delta_position = disp_between_pts(current_position,target);

    bool localsign = delta_position > 0 ? true : false;
    if (!pid.start && pid.sign != localsign) pid.oscillated++;
    pid.sign = localsign;

    //if (pid.oscillated > 0 && axis == 'y') delta_position = target.y - position.y;
    //if (pid.oscillated > 0 && axis == 'x') delta_position = target.x - position.x;
    
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


bool reversing = false;

void initialize() {
    left_encoder.reset_position();
    left_encoder.set_data_rate(5);
    back_encoder.reset_position();
    back_encoder.set_data_rate(5);
    //inertial_sensor.reset();
    inertial_sensor.set_heading(0);
    inertial_sensor.set_data_rate(5);

    left_drive_motors.tare_position_all();
    right_drive_motors.tare_position_all();

    pros::Task odom_task(track_robot);
}

void disabled() {}

// after init when in comp
void competition_initialize() {}

void turn_to_face(double heading, PID pid) {
    double turn_pid_out = 127;
    pid.start = true;

    while (fabs(turn_pid_out) > 0.6) {
        turn_pid_out = turn_pid(pid, heading);
        if (fabs(turn_pid_out) <= 0.6) break;

        turn_pid_out = fabs(turn_pid_out) < 12 ? (turn_pid_out > 0 ? 12 : -12) : turn_pid_out;
        turn_pid_out = fabs(turn_pid_out) > 60 ? (turn_pid_out > 0 ? 60 : -60) : turn_pid_out;

        left_drive_motors.move(turn_pid_out);
        right_drive_motors.move(turn_pid_out);

        pros::delay(5);
    }

    left_drive_motors.move(0); right_drive_motors.move(0);
}

void move_one_dir(double pos, PID pid, char axis, int dir = 1) {
    double linear_pid_out = 127;
    pid.start = true;

    while (fabs(linear_pid_out) > 0.75) {
        linear_pid_out = move_pid_one_dir(pid, pos, axis);
        std::cout << "x: " << position.x << ", y: " << position.y << ", theta: " << position.theta << std::endl;

        linear_pid_out = fabs(linear_pid_out) < 5 && fabs(linear_pid_out) > 0.75 ? (linear_pid_out > 0 ? 5 : -5) : linear_pid_out;
        linear_pid_out = fabs(linear_pid_out) > 60 ? (linear_pid_out > 0 ? 60 : -60) : linear_pid_out;

        left_drive_motors.move_velocity(linear_pid_out * dir);
        right_drive_motors.move_velocity(-linear_pid_out * dir);

        pros::delay(5);
    }

    left_drive_motors.move(0); right_drive_motors.move(0);
}

void move_to_point_straight(Point target, PID movepid, PID turnpid, char axis, int mult = 1, double heading = position.theta) {
    double abs_turn_to = atan2f(target.x - position.x, target.y - position.y) * (180/M_PI);
    if (abs_turn_to < 0) abs_turn_to += 360;

    double turn_pid_out = 127;
    double linear_pid_out = 127;

    turnpid.start = true;
    movepid.start = true;
    while (fabs(turn_pid_out) > 0.6 || fabs(linear_pid_out) > 0.25) {
        std::cout << "x: " << position.x << ", y: " << position.y << ", theta: " << position.theta << std::endl;
        double dx = target.x - position.x;
        double dy = target.y - position.y;
        if (fabs(dx) + fabs(dy) > 3) {
            abs_turn_to = atan2f(dx, dy) * (180/M_PI);
            if (abs_turn_to < 0) abs_turn_to += 360;
        }

        turn_pid_out = turn_pid(turnpid, abs_turn_to);
        //if (fabs(turn_pid_out) <= 0.6) break;

        //turn_pid_out = fabs(turn_pid_out) < 12 && fabs(turn_pid_out) > 0.6 ? (turn_pid_out > 0 ? 12 : -12) : turn_pid_out;
        turn_pid_out = fabs(turn_pid_out) > 60 ? (turn_pid_out > 0 ? 60 : -60) : turn_pid_out;


        linear_pid_out = move_pid(movepid, target, axis);
        //if (fabs(linear_pid_out) <= 0.25) break;

        linear_pid_out = fabs(linear_pid_out) < 10 && fabs(linear_pid_out) > 0.25 ? (linear_pid_out > 0 ? 10 : -10) : linear_pid_out;
        linear_pid_out = fabs(linear_pid_out) > 60 ? (linear_pid_out > 0 ? 60 : -60) : linear_pid_out;

        left_drive_motors.move_velocity(turn_pid_out + linear_pid_out*mult);
        right_drive_motors.move_velocity(turn_pid_out -linear_pid_out*mult);

        pros::delay(5);
    }

    turn_to_face(0, turnpid);
}

// INCREASE THREASHOLDS, MERGE CODE, CRY!

PID movepid = PID(1.25, 4.5, 0.15, 50, 5);
PID turnpid = PID(0.75, 0.1, 0.05, 45, 5);

void autonomous() {
    left_drive_motors.tare_position_all();
    right_drive_motors.tare_position_all();

    intake_motor.move_velocity(200);
    pros::delay(1500);
    move_one_dir(-11, movepid, 'y');
    pros::delay(1000);
    turn_to_face(270, turnpid);
    pros::delay(500);
    move_to_point_straight(Point(-13, -13), movepid, turnpid, 'x', -1, 270);
    mogo_piston.set_value(true);
}

void opcontrol() {
    while (inertial_sensor.is_calibrating()) pros::delay(5);
    left_drive_motors.set_brake_mode(pros::MotorBrake::coast);
    right_drive_motors.set_brake_mode(pros::MotorBrake::coast);

    Drive drive;
    bool mogo_state = false;

    int lift_target_pos = 0;
    bool move_lift_down = false;

    bool claw_lifted = false;

    int count = 0;

    while (true) {
        drive.movement();

        if (controller.get_digital_new_press(DIGITAL_R1)) {
            mogo_state = !mogo_state;
            mogo_piston.set_value(mogo_state);
        }

        if (controller.get_digital(DIGITAL_L1))
            intake_motor.move(127);
        else if (controller.get_digital(DIGITAL_L2)
                && !reversing) intake_motor.move(-127);
        else if (!reversing) intake_motor.move(0);

        if (controller.get_digital(DIGITAL_RIGHT)) doinker_piston.set_value(true);
        else doinker_piston.set_value(false);

        if (controller.get_digital_new_press(DIGITAL_LEFT)) {
            move_to_point_straight(Point(0,0), movepid, turnpid, 'y', 0);
        }

        if (count == 500) {
            std::cout << "x: " << position.x << ", y: " << position.y << ", theta: " << position.theta << std::endl;
            count = 0;
        }
        count++;

        pros::delay(5);
    }
}
