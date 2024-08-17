#include "main.h"
#include <cmath>

void initialize() {}

void disabled() {}

// after init when in comp
void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({8, 7, 10});
	pros::MotorGroup right_mg({1, 2, 3});

    const float exponential_a = pow(10, (log10(127)/80));

	while (true) {
		int dir = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);

        if (dir <= 80) dir *= exponential_a;
        else dir = 127;

        if (turn <= 80) turn *= exponential_a;
        else turn = 127;

		left_mg.move(- turn - dir);
		right_mg.move(- turn + dir);

		pros::delay(5);
	}
}
