#include "main.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "robot.hpp"

double Arm_pid(PID pid, double pos, double target) {
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

PID movepid = PID(1.4, 4.5, 0.25, 50, 5);
PID turnpid = PID(0.8, 0.1, 0.05, 45, 5);
PID Armpid = PID(1.50, 0.2, 0.15, 50, 5);

bool can_autoclamp = true;

void autoclamp_auton() {
    while (true) {
        if (mogo_distance.get_distance() < 30 && can_autoclamp) mogo_piston.set_value(true);
        
        pros::delay(5);
    }
}

bool Within_tolerance(double current, double target, double tolerance) {
    if (current - tolerance < target && current + tolerance > target) return true;
    return false;
}

void lb_down() {
    const double lb_rest_pos = 360;
    const double lb_pickup_pos = 335;
    const double lb_score_pos = 232;
    const double lb_tolerance = 5;

    while (true) {
        double lb_pos = lb_rotation_sensor.get_position() / 100.0;
        while (lb_pos < 180) lb_pos += 360;

        if (Within_tolerance(lb_pos, lb_rest_pos, lb_tolerance)) {
            lb_motors.move(0);
            break;
        } else {
            lb_motors.move(1 * Arm_pid(Armpid, lb_pos, lb_rest_pos));
        }

        pros::delay(5);
    }
}

const int route_num = 1;

void Check_colour() {
    const bool blue = route_num == 2;
    optical_sensor.set_led_pwm(100); // 90 blue 150 red

    while (true) {
        if (((blue && optical_sensor.get_rgb().red > 150) || (!blue && optical_sensor.get_rgb().blue > 90))) {
            pros::delay(300);
            intake_motor.move(0);
            pros::delay(350);
            intake_motor.move(127);
        }
        
        pros::delay(10);
    }

    std::cout << "blue: " << optical_sensor.get_rgb().blue;
    std::cout << " green: " << optical_sensor.get_rgb().green;
    std::cout << " red: " << optical_sensor.get_rgb().red << std::endl;
}

void lil_rev() {
    while (true) {
        if (fabs(intake_motor.get_actual_velocity()) < 10) {
            intake_motor.move(-127);
            pros::delay(250);
            intake_motor.move(127);
        }
        pros::delay(20);
    }
}

void autonomous() {
    pros::Task autoclamp(autoclamp_auton);

    if (route_num == 0) { // skills
        intake_motor.move_velocity(200);
        pros::delay(850);
        intake_motor.move(0);
        move_one_dir(-13, movepid, 'y');
        turn_to_face(270, turnpid);
        move_one_dir(-19, movepid, 'x', -1);
        mogo_piston.set_value(true);
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
        mogo_piston.set_value(true);
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
        intake_motor.move(0);
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
        move_one_dir(30, movepid, 'x', 0.6);
        // mogo in corner ^
        turn_to_face(260, turnpid, 1);
        //move_to_point_straight(Point(-37,-116), movepid, turnpid, 'y', false, false, 1);
        //turn_to_face(250, turnpid);
        move_one_dir(-110, movepid, 'x', -1.4, 3000);
        mogo_piston.set_value(false);
        move_one_dir(-30, movepid, 'x', -1.4, 2000);

        autoclamp.remove();
    }

    else if (route_num == 1) {
        pros::Task check_colour_task(Check_colour);
        inertial_sensor.set_heading(180);
        position.theta = 180;

        position.x = 4; position.y = 4.5;

        const double lb_rest_pos = 360;
        const double lb_pickup_pos = 335;
        const double lb_score_pos = 232;
        const double lb_tolerance = 5;

        PID temp_movepid = PID(1.4, 4.5, 0.25, 50, 5);
        PID temp_turnpid = PID(0.8, 0.1, 0.05, 45, 5);

        move_one_dir(8, temp_movepid, 'y', -3);
        turn_to_face(245, temp_turnpid);

        while (true) {
            double lb_pos = lb_rotation_sensor.get_position() / 100.0;
            while (lb_pos < 180) lb_pos += 360;
            if (Within_tolerance(lb_pos, lb_score_pos, lb_tolerance)) {
                lb_motors.move(0);
                break;
            } else {
                lb_motors.move(-1 * Arm_pid(Armpid, lb_pos, lb_score_pos));
            }

            pros::delay(5);
        }

        turn_to_face(242, temp_turnpid); //
        move_one_dir(-34, temp_movepid, 'x', -0.9, 1750);
        mogo_piston.set_value(true);
        pros::delay(200);

        turn_to_face(34, temp_turnpid);
        intake_motor.move(127);
        pros::delay(500);
        pros::Task lil_rev_task(lil_rev);
        move_one_dir(-48, temp_movepid, 'x', 1, 1000);
        // ring 1 on mogo ^
        move_one_dir(-42, temp_movepid, 'x', 1.3, 1000);
        turn_to_face(30, temp_turnpid, 1.8);
        move_one_dir(-48.5, temp_movepid, 'x', 1.5, 1000);
        // ring 2 on mogo ^
        pros::delay(200);

        move_one_dir(-26, temp_movepid, 'y', 1.2, 1000);
        turn_to_face(320, temp_turnpid, 1.6);
        move_one_dir(-32, temp_movepid, 'x', -1.3);
        turn_to_face(160, turnpid);
        left_drive_motors.move(-50); right_drive_motors.move(50);
        pros::delay(150);

        intake_motor.move(0);
        autoclamp.remove();
        check_colour_task.remove();
        lil_rev_task.remove();
    }

    else if (route_num == 2) {
        pros::Task check_colour_task(Check_colour);
        inertial_sensor.set_heading(180);
        position.theta = 180;

        position.x = 4; position.y = 4.5;

        const double lb_rest_pos = 360;
        const double lb_pickup_pos = 335;
        const double lb_score_pos = 232;
        const double lb_tolerance = 5;

        PID temp_movepid = PID(1.4, 4.5, 0.25, 50, 5);
        PID temp_turnpid = PID(0.8, 0.1, 0.05, 45, 5);

        move_one_dir(8.7, temp_movepid, 'y', -3);
        turn_to_face(115, temp_turnpid); // 245

        while (true) {
            double lb_pos = lb_rotation_sensor.get_position() / 100.0;
            while (lb_pos < 180) lb_pos += 360;
            if (Within_tolerance(lb_pos, lb_score_pos, lb_tolerance)) {
                lb_motors.move(0);
                break;
            } else {
                lb_motors.move(-1 * Arm_pid(Armpid, lb_pos, lb_score_pos));
            }

            pros::delay(5);
        }

        turn_to_face(118, temp_turnpid); // 242
        move_one_dir(36, temp_movepid, 'x', 0.9, 1750);
        mogo_piston.set_value(true);
        pros::delay(200);

        turn_to_face(332, temp_turnpid); // 
        intake_motor.move(127);
        pros::delay(500);
        pros::Task lil_rev_task(lil_rev);
        move_one_dir(54, temp_movepid, 'x', -1, 1000);
        // ring 1 on mogo ^
        turn_to_face(320, temp_turnpid, 1.8); // 30
        move_one_dir(46, temp_movepid, 'x', -1.3, 1000);
        turn_to_face(330, temp_turnpid, 2.8); // 30
        move_one_dir(53.5, temp_movepid, 'x', -1.5, 1000);
        // ring 2 on mogo ^
        pros::delay(200);

        move_one_dir(-26, temp_movepid, 'y', 1.2, 1000);
        pros::delay(700);
        turn_to_face(40, temp_turnpid, 1.6); // 320
        move_one_dir(36, temp_movepid, 'x', 1.3);
        turn_to_face(203, turnpid); // 160

        left_drive_motors.move(-50); right_drive_motors.move(50);
        autoclamp.remove();
        check_colour_task.remove();
        lil_rev_task.remove();
    }

}
