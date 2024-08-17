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

		left_mg.move(dir - turn);
		right_mg.move(dir + turn);

		pros::delay(5);
	}
}
