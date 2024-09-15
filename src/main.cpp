#include "main.h"
#include "pros/misc.h"
#include "robot.hpp"
#include <cmath>

const double EulerConstant = std::exp(1.0);

class Drive {
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

void move_intake(int position, bool move_down) {
    if (move_down) {
        if (lift_rotation_sensor.get_position() < position) lift_motor.move(0);
        else lift_motor.move(-127);
    }
    else {
        if (lift_rotation_sensor.get_position() > position) lift_motor.move(0);
        else lift_motor.move(127);
    }
}

void initialize() {
    lift_motor.set_brake_mode(pros::MotorBrake::hold);
}

void disabled() {}

// after init when in comp
void competition_initialize() {}

void opcontrol() {
    Drive drive;
    bool mogo_state = false;

    int lift_target_pos = 0;
    bool move_lift_down = false;
    bool lift_is_down = true;

	while (true) {
        drive.movement();

        if (controller.get_digital_new_press(DIGITAL_R2) || controller.get_digital_new_press(DIGITAL_R1)) {
            mogo_state = !mogo_state;
            mogo_piston.set_value(mogo_state);
        }

        if (controller.get_digital(DIGITAL_L1)) intake_motor.move(127);
        else if (controller.get_digital(DIGITAL_L2)) intake_motor.move(-127);
        else intake_motor.move(0);

        if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
            if (lift_is_down) lift_target_pos = 6000;
            else lift_target_pos = 9000;
            move_lift_down = false;
        }
        if (controller.get_digital_new_press(DIGITAL_DOWN)) {
            lift_target_pos = 0;
            move_lift_down = true;
        }

        move_intake(lift_target_pos, move_lift_down);

		pros::delay(5);
	}
}

