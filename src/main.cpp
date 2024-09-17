#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include <cmath>

class Drive {
    const double EulerConstant = std::exp(1.0);
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

class Position {
    public: float x = 0;
            float y = 0;
            float theta = 0;
};

Position position;

void track_robot() {
    const float left_offset = 0.8267;
    const float back_offset = 2.2047;
    const float wheel_diameter = 2; // inches

    float last_imu_reading = 0;
    float last_left_reading = 0;
    float last_back_reading = 0;

    position.x = 0;
    position.y = 0;
    position.theta = 0;

    while (inertial_sensor.is_calibrating()) {
        pros::c::delay(10);
    }

	while (true) {
        float imu_reading = inertial_sensor.get_heading();
        if (imu_reading > 360) imu_reading = 0;
        float left_reading = left_encoder.get_position() / 100.0;
        float back_reading = back_encoder.get_position() / 100.0;

        float delta_imu = imu_reading - last_imu_reading;
        float delta_left_reading = left_reading - last_left_reading;
        float delta_back_reading = back_reading - last_back_reading;

        float delta_dist_left = M_PI*wheel_diameter * (delta_left_reading/360);
        float delta_dist_back = M_PI*wheel_diameter * (delta_back_reading/360);

        position.theta = imu_reading;
        float delta_theta = imu_reading - last_imu_reading;

        float delta_local_offset[2];
        if (fabs(delta_theta) < 0.0001) {
            delta_local_offset[0] = delta_dist_back;
            delta_local_offset[1] = delta_dist_left;
        }
        else {
            const float constant = 2*sinf((delta_theta * M_PI) / 360);
            delta_local_offset[0] = constant * ( (delta_dist_back/delta_theta) + back_offset );
            delta_local_offset[1] = constant * ( (delta_dist_left/delta_theta) + left_offset );
        }

        float avg_theta = last_imu_reading + (delta_theta / 2);

        float polar_delta_local_offset[2] = {
            sqrtf( delta_local_offset[0] * delta_local_offset[0] + delta_local_offset[1] * delta_local_offset[1] ),
            delta_local_offset[0] == 0
                ? (delta_local_offset[1] > 0) ? 90 : -90
                : atanf( delta_local_offset[1] / delta_local_offset[0] ) * (float) (180.0/M_PI)
        }; // distance, heading
        polar_delta_local_offset[1] += avg_theta;
        float delta_global_offset[2] = { // cartesian
            polar_delta_local_offset[0] * cosf(polar_delta_local_offset[1] * (M_PI / 180.0)),
            polar_delta_local_offset[0] * sinf(polar_delta_local_offset[1] * (M_PI / 180.0))
        };

        position.x += delta_global_offset[0];
        position.y += delta_global_offset[1];

        last_imu_reading = imu_reading;
        last_left_reading = left_reading;
        last_back_reading = back_reading;

        pros::delay(5);
    }
}

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

void test_odom() {
    while (inertial_sensor.is_calibrating()) pros::delay(100);
    for (int i = 0; i < 10; i++) {
        int counter = 0;
        while (position.y > -0.3) {
            counter++;
            left_drive_motors.move(-50);
            right_drive_motors.move(50);
            if (counter % 25 == 0) std::cout << position.y << "\n";
            pros::delay(5);
        }
        left_drive_motors.move(0);
        right_drive_motors.move(0);
        pros::delay(500);
        while (position.y > -0.6) {
            counter++;
            left_drive_motors.move(50);
            right_drive_motors.move(-50);
            if (counter % 25 == 0) std::cout << position.y << "\n";
            pros::delay(5);
        }
        left_drive_motors.move(0);
        right_drive_motors.move(0);
        pros::delay(500);
    }
}

void initialize() {
    left_encoder.reset_position();
    left_encoder.set_data_rate(5);
    back_encoder.reset_position();
    back_encoder.set_data_rate(5);
    inertial_sensor.reset();
    inertial_sensor.set_heading(0);
    inertial_sensor.set_data_rate(5);

    pros::Task odom_task(track_robot);

    lift_motor.set_brake_mode(pros::MotorBrake::hold);
}

void disabled() {}

// after init when in comp
void competition_initialize() {}

void opcontrol() {
    Drive drive;
    bool mogo_state = false;
    int count = 0;

    int lift_target_pos = 0;
    bool move_lift_down = false;
    bool lift_is_down = true;

    test_odom();

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
            if (lift_is_down) {
                lift_target_pos = 6000;
                lift_is_down = false;
            }
            else lift_target_pos = 9000;
            move_lift_down = false;
        }
        if (controller.get_digital_new_press(DIGITAL_DOWN)) {
            lift_target_pos = 0;
            move_lift_down = true;
            lift_is_down = true;
        }

        move_intake(lift_target_pos, move_lift_down);

        pros::delay(5);
    }
}
