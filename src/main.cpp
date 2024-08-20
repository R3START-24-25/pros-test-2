#include "main.h"

void initialize() {}

void disabled() {}

// after init when in comp
void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({8, 9, 10});
	pros::MotorGroup right_mg({1, 2, 3});

	while (true) {
		int dir = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
    
    if (turn <= 80) turn *= exponential_a;
    else if (turn > 0) turn = 127;
    else if (turn >= -80) turn = abs(turn) * exponential_a * -1;
    else if (turn < 0) turn = -127;

    if (dir > 2) dir = exponential_b(dir);
    else if (dir < -2) dir = -1*exponential_b(abs(dir));
    else dir = 0;

		left_mg.move(- turn - dir);
		right_mg.move(- turn + dir);

		pros::delay(5);
	}
}
