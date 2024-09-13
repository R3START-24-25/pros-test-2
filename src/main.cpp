#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
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

class Position {
    public: float x = 0;
            float y = 0;
            float theta = 0;
};

Position position;

class Odom {
    const float left_offset = 0.8267;
    const float back_offset = 2.2047;
    const float wheel_diameter = 2; // inches

    float last_imu_reading = 0;
    float last_left_reading = 0;
    float last_back_reading = 0;

    public: void update_position() {
        while(true) {
            float imu_reading = inertial_sensor.get_heading();
            float left_reading = left_encoder.get_position() / 100.0;
            float back_reading = back_encoder.get_position() / 100.0;

            float delta_imu = imu_reading - last_imu_reading;
            float delta_left_reading = left_reading - last_left_reading;
            float delta_back_reading = back_reading - last_back_reading;

            float delta_dist_left = M_PI*wheel_diameter * (delta_left_reading/360);
            float delta_dist_back = M_PI*wheel_diameter * (delta_back_reading/360);

            position.theta = imu_reading;
            float delta_theta = imu_reading - last_imu_reading;

            last_imu_reading = imu_reading;
            last_left_reading = left_reading;
            last_back_reading = back_reading;

            float delta_local_offset[2];
            if (delta_theta == 0) {
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
                atanf( delta_local_offset[1] / delta_local_offset[0] )
            }; // distance, heading
            polar_delta_local_offset[1] += avg_theta;
            float delta_global_offset[2] = { // cartesian
                polar_delta_local_offset[0] * cosf(polar_delta_local_offset[1]),
                polar_delta_local_offset[0] * sinf(polar_delta_local_offset[1])
            };

            position.x += delta_global_offset[0];
            position.y += delta_global_offset[1];

            pros::c::delay(10);
        }
    }
};

void initialize() {
    inertial_sensor.reset();
    inertial_sensor.set_heading(0);

    left_encoder.reset_position();
    back_encoder.reset_position();

    Odom odom;
    pros::Task odom_thread( [&odom]() {odom.update_position();} );
}

void disabled() {}

// after init when in comp
void competition_initialize() {}

void opcontrol() {
    Drive drive;
    bool mogo_state = false;
    int count = 0;

    while (true) {
        drive.movement();

        if (controller.get_digital_new_press(DIGITAL_R2) || controller.get_digital_new_press(DIGITAL_R1)) {
            mogo_state = !mogo_state;
            mogo_piston.set_value(mogo_state);
        }

        if (count % 500 == 0) {
            std::cout << "x: " << position.x << ", y: " << position.y << ", theta: " << position.theta << "\n";
        }
        count++;

        pros::delay(5);
    }
}
