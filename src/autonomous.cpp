#include "main.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "robot.hpp"

PID movepid = PID(1.4, 4.5, 0.25, 50, 5);
PID turnpid = PID(0.8, 0.1, 0.05, 45, 5);

bool can_autoclamp = true;

void autoclamp_auton() {
    while (true) {
        if (mogo_distance.get_distance() < 30 && can_autoclamp) mogo_piston.set_value(true);
    }
}

const int route_num = 1;

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
        PID temp_movepid = PID(1.4, 4.5, 0.25, 50, 5);
        PID temp_turnpid = PID(0.8, 0.1, 0.05, 45, 5);

        move_one_dir(16.5, temp_movepid, 'y', 0.85);
        turn_to_face(85, temp_turnpid, 1);

        left_drive_motors.move(25); right_drive_motors.move(-25);
        pros::delay(150);
        while (abs(left_encoder.get_velocity()) > 200) pros::delay(5);
        left_drive_motors.move(0); right_drive_motors.move(0);

        intake_motor.move(127);
        pros::delay(900);
        intake_motor.move(0);
        move_one_dir(-2, temp_movepid, 'x', 1.1);
        turn_to_face(219, temp_turnpid);
        move_one_dir(-34, temp_movepid, 'x', -1.4, 1750);
        //mogo_piston.set_value(true);
        pros::delay(200);

        turn_to_face(34, temp_turnpid);
        intake_motor.move(127);
        move_one_dir(-44, temp_movepid, 'x', 1.25, 1000);
        // ring 1 on mogo ^
        turn_to_face(1, turnpid);
        move_one_dir(-36, movepid, 'y', 2.5);
        pros::delay(200);
        /*
        move_one_dir(-42, temp_movepid, 'x', 1.3, 1000);
        turn_to_face(30, temp_turnpid, 1.8);
        move_one_dir(-48, temp_movepid, 'x', 1.5, 1000);
        */

        move_one_dir(-26, temp_movepid, 'y', 1.7, 1000);
        turn_to_face(320, temp_turnpid, 2.3);
        can_autoclamp = false;
        mogo_piston.set_value(false);
        move_one_dir(-34, temp_movepid, 'x', -2.5);
        can_autoclamp = true;
        intake_motor.move(0);
        turn_to_face(180, temp_turnpid);
        move_to_point_straight(Point(-15,28), temp_movepid, temp_turnpid, 'x', true, false, -2.5, 290, 1750, false);
        move_one_dir(-25, temp_movepid, 'x', -3);
        mogo_piston.set_value(true);
        intake_motor.move(127);
        turn_to_face(0, turnpid);
        move_one_dir(0, movepid, 'y');
        /*
        */
    }

    autoclamp.remove();
}
