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
    p_i_d = p_i_d > 65 ? 65 : p_i_d;
    return p_i_d;
}

PID movepid = PID(1.4, 4.5, 0.25, 50, 5);
PID turnpid = PID(0.8, 0.1, 0.05, 45, 5);
PID Armpid = PID(1.75, 0.3, 0.15, 50, 5);

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
    const double lb_pickup_pos = 337.5;
    const double lb_score_pos = 235;
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

const int route_num = 4;

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

bool can_sort = true;
void colour_sort() {
    optical_sensor.set_led_pwm(255);
    bool blue = route_num == 5;
    while (true) {
        if ( can_sort && ((blue && optical_sensor.get_rgb().red > 150) || (!blue && optical_sensor.get_rgb().blue > 110)) ) {
            pros::delay(250);
            intake_motor.move(-127);
            pros::delay(70);
            intake_motor.move(127);
        }
        pros::delay(10);
    }
}

void red_sort() {
    while (true) {
        if (optical_sensor.get_proximity() > 140) {
            intake_motor.move(0);
            can_sort = false;
        }
        pros::delay(10);
    }
}

void quick_rev() {
    while (true) {
        if (intake_motor.get_actual_velocity() < 15 && intake_motor.get_power() > 5) {
            intake_motor.move(-127);
            pros::delay(150);
            intake_motor.move(127);
        }
        pros::delay(10);
    }
}

void lb_raise() {
    const double lb_rest_pos = 360;
    const double lb_pickup_pos = 337.5;
    const double lb_score_pos = 235;
    const double lb_doublescore_pos = 225;
    const double lb_tolerance = 2.5;

    while (true) {
        double lb_pos = lb_rotation_sensor.get_position() / 100.0;
        while (lb_pos < 180) lb_pos += 360;
        if (Within_tolerance(lb_pos, lb_score_pos, lb_tolerance)) {
            lb_motors.move(0);
        } else {
            lb_motors.move(-1 * Arm_pid(Armpid, lb_pos, lb_score_pos));
        }

        pros::delay(5);
    }
}

void autonomous() {
    pros::Task autoclamp(autoclamp_auton);
    pros::Task quickrev(quick_rev);

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

    else if (route_num == 4) { // new red awp
        pros::Task coloursort(colour_sort);

        PID new_movepid = PID(2.0, 4.5, 0.25, 50, 5);
        PID new_turnpid = PID(0.6, 0.1, 0.00, 45, 5);

        inertial_sensor.set_heading(29);
        inertial_sensor.set_rotation(29);
        position.theta = 29;

        move_one_dir_advanced(Point(20, 27), new_movepid, new_turnpid, 'y', 0.8);
        mogo_piston.set_value(true);
        // pick up mogo ^
        move_one_dir_advanced(Point(20, 23.5), new_movepid, new_turnpid, 'y', 1.5);
        intake_motor.move(127);
        turn_to_face_new(130, new_turnpid, 1.2);
        move_one_dir_advanced(Point(5, 40.5), new_movepid, new_turnpid, 'y', -1.2);
        // pick up ring 1 ^
        turn_to_face_new(150, new_turnpid, 1.5);
        move_one_dir_advanced(Point(5, 35), new_movepid, new_turnpid, 'y', -1.7);
        // reverse out ^
        turn_to_face_new(140, new_turnpid, 1.5);
        move_one_dir_advanced(Point(-8, 40.5), new_movepid, new_turnpid, 'y', -1.4);
        pros::delay(400);
        // 2 discs picked up ^^
        move_one_dir_advanced(Point(0, 37), new_movepid, new_turnpid, 'x', 1.7);
        turn_to_face_new(60, turnpid, 1.2);
        pros::delay(400);
        coloursort.remove();
        pros::Task redsort(red_sort);
        move_one_dir_advanced(Point(-5, 33), new_movepid, new_turnpid, 'x', 1.7);

        turn_to_face_new(340, turnpid, 2);
        left_drive_motors.move(-70); right_drive_motors.move(70);
        pros::delay(350);
        mogo_piston.set_value(false);

        move_to_point_straight(Point(36, -1), new_movepid, new_turnpid, 'x', false, false, 1.1, 0, 1000);
        move_one_dir_advanced(Point(37.1, -1), new_movepid, new_turnpid, 'x', -0.9);
        turn_to_face_new(186, turnpid, 1.2);

        redsort.remove();
        pros::Task lbup(lb_raise);

        left_drive_motors.move(25); right_drive_motors.move(-25);
        pros::delay(200);
        while (abs(left_encoder.get_velocity()) > 100) pros::delay(5);
        pros::delay(100);
        left_drive_motors.move(80); right_drive_motors.move(-80);
        pros::delay(750);
        left_drive_motors.move(0); right_drive_motors.move(0);
        pros::delay(200);

        intake_motor.move(127);
        pros::delay(1000);
        intake_motor.move(0);

        left_drive_motors.move(-75); right_drive_motors.move(75);
        pros::delay(200);
        while (abs(left_encoder.get_velocity()) > 100) pros::delay(5);
        pros::delay(100);
        left_drive_motors.move(0); right_drive_motors.move(0);

        left_drive_motors.move(-30); right_drive_motors.move(-30);
        pros::delay(200);
        left_drive_motors.move(0); right_drive_motors.move(0);

        lbup.remove();
    }

    else if (route_num == 5) { // new blue awp
        pros::Task coloursort(colour_sort);

        PID new_movepid = PID(2.0, 4.5, 0.25, 50, 5);
        PID new_turnpid = PID(0.6, 0.1, 0.00, 45, 5);

        inertial_sensor.set_heading(331); // 360 - 29
        inertial_sensor.set_rotation(331);
        position.theta = 331;

        move_one_dir_advanced(Point(-20, 27), new_movepid, new_turnpid, 'y', 0.8);
        mogo_piston.set_value(true);
        // pick up mogo ^
        move_one_dir_advanced(Point(-20, 23.5), new_movepid, new_turnpid, 'y', 1.5);
        intake_motor.move(127);
        turn_to_face_new(230, new_turnpid, 1.2); // 360 - 130
        move_one_dir_advanced(Point(-5, 40.5), new_movepid, new_turnpid, 'y', -1.2);
        // pick up ring 1 ^
        turn_to_face_new(210, new_turnpid, 1.5); // 360 - 150
        move_one_dir_advanced(Point(-5, 35), new_movepid, new_turnpid, 'y', -1.7);
        // reverse out ^
        turn_to_face_new(220, new_turnpid, 1.5); // 360 - 140
        move_one_dir_advanced(Point(8, 40.5), new_movepid, new_turnpid, 'y', -1.4);
        pros::delay(400);
        // 2 discs picked up ^^
        move_one_dir_advanced(Point(0, 37), new_movepid, new_turnpid, 'x', 1.7);
        turn_to_face_new(300, turnpid, 1.2); // 360 - 60
        pros::delay(400);
        coloursort.remove();
        pros::Task redsort(red_sort);
        move_one_dir_advanced(Point(5, 33), new_movepid, new_turnpid, 'x', 1.7);

        turn_to_face_new(20, turnpid, 2); // 360 - 340
        left_drive_motors.move(-70); right_drive_motors.move(70);
        pros::delay(350);
        mogo_piston.set_value(false);

        move_to_point_straight(Point(-36, -1), new_movepid, new_turnpid, 'x', false, false, 1.1, 0, 1000);
        move_one_dir_advanced(Point(-37.1, -1), new_movepid, new_turnpid, 'x', -0.9);
        turn_to_face_new(174, turnpid, 1.2); // 360 - 186

        redsort.remove();
        pros::Task lbup(lb_raise);

        left_drive_motors.move(25); right_drive_motors.move(-25);
        pros::delay(200);
        while (abs(left_encoder.get_velocity()) > 100) pros::delay(5);
        pros::delay(100);
        left_drive_motors.move(80); right_drive_motors.move(-80);
        pros::delay(750);
        left_drive_motors.move(0); right_drive_motors.move(0);
        pros::delay(200);

        intake_motor.move(127);
        pros::delay(1000);
        intake_motor.move(0);

        left_drive_motors.move(-75); right_drive_motors.move(75);
        pros::delay(200);
        while (abs(left_encoder.get_velocity()) > 100) pros::delay(5);
        pros::delay(100);
        left_drive_motors.move(0); right_drive_motors.move(0);

        left_drive_motors.move(-30); right_drive_motors.move(-30);
        pros::delay(200);
        left_drive_motors.move(0); right_drive_motors.move(0);

        lbup.remove();
    }

    quickrev.remove();
}
