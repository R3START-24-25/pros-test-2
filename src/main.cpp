#include "main.h"
#include "pros/misc.h"
#include "robot.hpp"
#include <cmath>

const double EulerConstant = std::exp(1.0);

void initialize() {}

void disabled() {}

// after init when in comp
void competition_initialize() {}

//azlan comment c++
void autonomous() {}

float exponential_b(float x) {
    return ((1/(1+pow(EulerConstant, (-1*(x-83.5)/12))))*110)+20;
}

void opcontrol() {
    const float exponential_a = pow(10, (log10(127)/80));
    bool mogo_state = false;

	while (true) {
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

        if (controller.get_digital_new_press(DIGITAL_R2) || controller.get_digital_new_press(DIGITAL_R1)) {
            mogo_state = !mogo_state;
            mogo_piston.set_value(mogo_state);
        }

		pros::delay(5);
	}
}
