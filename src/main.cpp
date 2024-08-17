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

        //if (dir <= 80) dir *= exponential_a;
        //else if (dir > 0) dir = 127;
        //else if (dir >= -80) dir = abs(dir) * exponential_a * -1;
        //else if (dir < 0) dir = -127;

        //if (turn <= 80) turn *= exponential_a;
        //else if (turn > 0) turn = 127;
        //else if (turn >= -80) turn = abs(turn) * exponential_a * -1;
        //else if (turn < 0) turn = -127;
        
        if (dir > 2 && dir < 35) dir = 20;
        else if (dir > 90) dir = 127;
        else if (dir < -2 && dir > -35) dir = -20;
        else if (dir < -90) dir = -127;
        
        if (turn > 2 && turn < 35) turn = 20;
        else if (turn > 90) turn = 127;
        else if (turn < -2 && turn > -35) turn = -20;
        else if (turn < -90) turn = -127;

		left_mg.move(- turn - dir);
		right_mg.move(- turn + dir);

		pros::delay(5);
	}
}
