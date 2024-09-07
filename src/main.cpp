#include "main.h"
#include "pros/misc.h"
#include <cmath>

const double EulerConstant = std::exp(1.0);

void initialize() {}

void disabled() {}

// after init when in comp
void competition_initialize() {}

void autonomous() {}

float exponential_b(float x) {
    return ((1/(1+pow(EulerConstant, (-1*(x-83.5)/12))))*110)+20;
}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({8, 7, 10});
	pros::MotorGroup right_mg({1, 2, 3});
    pros::adi::DigitalOut mogo_mech(1);

    const float exponential_a = pow(10, (log10(127)/80));
    bool mogo_state = false;

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

		left_mg.move(-turn - dir);
		right_mg.move(-turn + dir);

        if (master.get_digital_new_press(DIGITAL_R2) || master.get_digital_new_press(DIGITAL_R1)) {
            mogo_state = !mogo_state;
            mogo_mech.set_value(mogo_state);
        }

		pros::delay(5);
	}
}
