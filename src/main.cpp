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

double arm_pid(PID pid, double pos, double target) {
    if (pid.start) {
        pid.last_delta_position = 0;
        pid.oscillated = 0;
        pid.integral = 0;
        pid.start = false;
    }

    double delta_position = fabs(target - pos);

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

    bool localsign = delta_position > 0 ? true : false;
    if (!pid.start && pid.sign != localsign) pid.oscillated++;
    pid.sign = localsign;

    //if (pid.oscillated > 0 && axis == 'y') delta_position = target.y - position.y;
    //if (pid.oscillated > 0 && axis == 'x') delta_position = target.x - position.x;
    
    pid.integral = fabs(delta_position) > 2.5 && delta_position < pid.integral_threshold
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

    lb_motors.set_brake_mode(pros::MotorBrake::hold);
}

void disabled() {}

// after init when in comp
void competition_initialize() {}

void turn_to_face(double heading, PID pid, double mult = 1) {
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
    }

    left_drive_motors.move(0); right_drive_motors.move(0);
}

void move_one_dir(double pos, PID pid, char axis, double dir = 1, int timeout = 2000) {
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

void move_to_point_straight(Point target, PID movepid, PID turnpid, char axis, bool turn, bool xrev = false, double mult = 1, double heading = position.theta, int timeout = 3000) {
    int time = 0;
    double abs_turn_to = atan2f(target.x - position.x, target.y - position.y) * (180/M_PI);
    if (abs_turn_to < 0) abs_turn_to += 360;

    double turn_pid_out = 127;
    double linear_pid_out = 127;

    turnpid.start = true;
    movepid.start = true;
    while (fabs(turn_pid_out) > 0.6 || fabs(linear_pid_out) > 0.25) {
        double dx = target.x - position.x; //30
        double dy = target.y - position.y; //-5
        if (fabs(dx) + fabs(dy) > 3) {
            abs_turn_to = atan2f(dx, dy) * (180/M_PI); //100
            if (abs_turn_to < 0) abs_turn_to += 360;
            //if (xrev) abs_turn_to = fmod(abs_turn_to + 180, 360);
            abs_turn_to = fmod(abs_turn_to + 180, 360);
            std::cout << "abs_turn_to: " << abs_turn_to << std::endl;
        }

        turn_pid_out = turn_pid(turnpid, abs_turn_to); //100

        //turn_pid_out = fabs(turn_pid_out) < 12 && fabs(turn_pid_out) > 0.6 ? (turn_pid_out > 0 ? 12 : -12) : turn_pid_out;
        turn_pid_out = fabs(turn_pid_out) > 60 ? (turn_pid_out > 0 ? 60 : -60) : turn_pid_out;


        linear_pid_out = move_pid(movepid, target, axis, xrev);

        linear_pid_out = fabs(linear_pid_out) < 10 && fabs(linear_pid_out) > 0.25 ? (linear_pid_out > 0 ? 10 : -10) : linear_pid_out;
        linear_pid_out = fabs(linear_pid_out) > 60 ? (linear_pid_out > 0 ? 60 : -60) : linear_pid_out;

        if (xrev) linear_pid_out *= 1;
        if (xrev) turn_pid_out = 0;
        
        left_drive_motors.move((turn_pid_out + linear_pid_out)*mult);
        right_drive_motors.move((turn_pid_out - linear_pid_out)*mult);

        pros::delay(5);
        time += 5;

        if (time >= timeout) break;
    }

    if (turn) turn_to_face(heading, turnpid);
}

// INCREASE THREASHOLDS, MERGE CODE, CRY!

PID movepid = PID(1.4, 4.5, 0.25, 50, 5);
PID turnpid = PID(0.8, 0.1, 0.05, 45, 5);
PID armpid  = PID(1.50, 0.2, 0.15, 50, 5);

void autoclamp_auton() {
    while (true) {
        if (mogo_distance.get_distance() < 30) mogo_piston.set_value(true);
    }
}

bool within_tolerance(double current, double target, double tolerance) {
    if (current - tolerance < target && current + tolerance > target) return true;
    return false;
}

const int route_num = 0;

void autonomous() {
    pros::Task autoclamp(autoclamp_auton);

    if (route_num == 0) { // skills
        intake_motor.move_velocity(200);
        pros::delay(850);
        intake_motor.move(0);
        move_one_dir(-13, movepid, 'y');
        turn_to_face(270, turnpid);
        move_one_dir(-19, movepid, 'x', -1);
        //mogo_piston.set_value(true);
        turn_to_face(0, turnpid);
        intake_motor.move_velocity(200);
        move_one_dir(-37, movepid, 'y');
        turn_to_face(84, turnpid);
        move_one_dir(-48, movepid, 'x');
        turn_to_face(174, turnpid); // 180
        move_one_dir(-16, movepid, 'y', -1);
        move_one_dir(-3, movepid, 'y', -0.7);
        move_one_dir(-6, movepid, 'y', -1.3);
        turn_to_face(60, turnpid);
        move_one_dir(-15, movepid, 'y', 1.1);
        turn_to_face(340, turnpid);
        move_one_dir(-7, movepid, 'y', 1.1, 1000);
        mogo_piston.set_value(false);
        // mogo in corner
        move_one_dir(-13, movepid, 'y', 1.6);
        turn_to_face(270, turnpid, 1.4);

        intake_motor.move(0);

        move_to_point_straight(Point(6,-16.5), movepid, turnpid, 'x', true, false, 1, 90, 2500);
        move_one_dir(23, movepid, 'x');
        // picked up mogo ^
        turn_to_face(5, turnpid);
        intake_motor.move_velocity(200);
        move_one_dir(-41, movepid, 'y');
        // ring 1 ^
        turn_to_face(275, turnpid);
        move_one_dir(48, movepid, 'x', -1);
        // ring 2 ^
        turn_to_face(184, turnpid); // 180 //
        move_one_dir(-16, movepid, 'y', -1);
        move_one_dir(-4, movepid, 'y', -0.7);
        move_one_dir(-6, movepid, 'y', -1.3);
        // rings 3/4 ^
        turn_to_face(300, turnpid);
        move_one_dir(-18, movepid, 'y', 1.1);
        // ring 5 ^
        turn_to_face(30, turnpid);
        move_one_dir(-6, movepid, 'y', 1.1, 1000);
        mogo_piston.set_value(false);
        // mogo in corner
        move_one_dir(-13, movepid, 'y', 1.6);

        // go to blue mogo
        move_to_point_straight(Point(35,-108), movepid, turnpid, 'y', true, false, 1, 206);
        move_one_dir(-132, turnpid, 'y', -2.5);
        mogo_piston.set_value(true);
        move_one_dir(-125, turnpid, 'y', -2.5);
        turn_to_face(140, turnpid);
        move_one_dir(80, movepid, 'x', 1.4, 2000);
        pros::delay(500);
        mogo_piston.set_value(false);
        move_one_dir(60, movepid, 'x', -0.6);
        // mogo in corner ^

    }

    else if (route_num == 1) {
        move_one_dir(15.6, movepid, 'y', 1);
        turn_to_face(90, turnpid);

        left_drive_motors.move(30); right_drive_motors.move(-30);
        pros::delay(150);
        while (abs(left_encoder.get_velocity()) > 200) pros::delay(5);
        left_drive_motors.move(0); right_drive_motors.move(0);

        intake_motor.move(127);
        pros::delay(900);
        intake_motor.move(0);
        move_one_dir(-2, movepid, 'x', 1);
        turn_to_face(223, turnpid);
        move_one_dir(-31, movepid, 'x', -1.5, 1750);
        //mogo_piston.set_value(true);
        pros::delay(100);

        turn_to_face(35, turnpid);
        intake_motor.move(127);
        move_one_dir(-47, movepid, 'x', 1.3, 1000);
        move_one_dir(-44, movepid, 'x', 1.3, 1000);
        turn_to_face(22, turnpid, 1.8);
        move_one_dir(-47.5, movepid, 'x', 1.5, 1000);

        move_one_dir(-42, movepid, 'x', 1.7, 1000);
        mogo_piston.set_value(false);
        turn_to_face(320, turnpid, 2.3);
        move_one_dir(-32, movepid, 'x', -1.5);
        move_to_point_straight(Point(-18,32), movepid, turnpid, 'x', false, false, 4.8, 1500);
    }

    autoclamp.remove();
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
    int last_unclamp = 0;
    int last_autoclamp = -100;
    bool can_autoclamp = true;

    const double lb_rest_pos = 360;
    const double lb_pickup_pos = 335;
    const double lb_score_pos = 232;
    const double lb_tolerance = 5;
    int lb_state = 0; // 0 = still, 1 = move to rest, 2 = move to pickup, 3 = move to score
    int lb_dir = 1;

    while (true) {
        drive.movement();

        if (controller.get_digital_new_press(DIGITAL_R1) && count >= last_autoclamp + 100) {
            mogo_state = !mogo_state;
            mogo_piston.set_value(mogo_state);
            if (!mogo_state) last_unclamp = count;
            controller.rumble("_");
            can_autoclamp = false;
        }
        if (mogo_distance.get_distance() < 30 && can_autoclamp) {
            mogo_state = true;
            mogo_piston.set_value(mogo_state);
            controller.rumble("-");
            last_autoclamp = count;
        }
        if (mogo_distance.get_distance() > 40 && count > last_unclamp + 200) can_autoclamp = true;

        if (controller.get_digital(DIGITAL_L1))
            intake_motor.move(127);
        else if (controller.get_digital(DIGITAL_L2)
                && !reversing) intake_motor.move(-127);
        else if (!reversing) intake_motor.move(0);

        if (controller.get_digital(DIGITAL_RIGHT)) doinker_piston.set_value(true);
        else doinker_piston.set_value(false);

        if (controller.get_digital_new_press(DIGITAL_LEFT)) {
            move_to_point_straight(Point(0,0), movepid, turnpid, true, 'y', 0);
        }

        double lb_pos = lb_rotation_sensor.get_position() / 100.0;
        while (lb_pos < 180) lb_pos += 360;

        if (controller.get_digital_new_press(DIGITAL_R2)) {
            if (within_tolerance(lb_pos, lb_pickup_pos, lb_tolerance)) {
                lb_state = 1;
                lb_dir = 1;
            } else if (lb_state == 2 && lb_dir == 1) {
                lb_state = 1;
            } else {
                lb_state = 2;
                if (lb_pos < lb_pickup_pos) lb_dir = 1;
                else lb_dir = -1;
            }
        }
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            if (within_tolerance(lb_pos, lb_pickup_pos, lb_tolerance)) {
                lb_state = 3;
                lb_dir = -1;
            }
        }
                
        switch (lb_state) {
            case 0:
                break;
            case 1:
                if (within_tolerance(lb_pos, lb_rest_pos, lb_tolerance)) {
                    lb_motors.move_velocity(0);
                    lb_state = 0;
                } else {
                    lb_motors.move(lb_dir * arm_pid(armpid, lb_pos, lb_rest_pos));
                }
                break;
            case 2:
                if (within_tolerance(lb_pos, lb_pickup_pos, lb_tolerance)) {
                    lb_motors.move_velocity(0);
                    lb_state = 0;
                } else {
                    lb_motors.move(lb_dir * arm_pid(armpid, lb_pos, lb_pickup_pos));
                }
                break;
            case 3:
                if (within_tolerance(lb_pos, lb_score_pos, lb_tolerance)) {
                    lb_motors.move_velocity(0);
                    lb_state = 0;
                } else {
                    lb_motors.move(lb_dir * arm_pid(armpid, lb_pos, lb_score_pos));
                }
                break;
        }

        if (count % 500 == 0) {
            std::cout << "x: " << position.x << ", y: " << position.y << ", theta: " << position.theta << std::endl;
            std::cout << "lb pos: " << lb_pos << std::endl;
        }
        count++;

        pros::delay(5);
    }
}
