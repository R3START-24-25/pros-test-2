#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "robot.hpp"

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

double arm_pid(PID pid, double pos, double target) {
    if (pid.start) {
        pid.last_delta_position = 0;
        pid.oscillated = 0;
        pid.integral = 0;
        pid.start = false;
    }

    double delta_position = fabs(target - pos);

    pid.integral = fabs(delta_position) > 2.5 && delta_position < pid.integral_threshold
        ? pid.integral + delta_position
        : 0;
    
    float derivative = delta_position - pid.last_delta_position;
    pid.last_delta_position = delta_position;
    if (pid.start) {
        derivative = 0;
        pid.start = false;
    }

    float p_i_d = (delta_position * pid.kP) + (pid.integral * pid.kI) + (derivative * pid.kD);
    return p_i_d;
}

bool reversing = false;
bool can_intake = true;

void check_colour() {
    const bool blue = false;
    optical_sensor.set_led_pwm(100); // 90 blue 150 red

    while (true) {
        if (((blue && optical_sensor.get_rgb().red > 150) || (!blue && optical_sensor.get_rgb().blue > 90)) && controller.get_digital(DIGITAL_L1)) {
            pros::delay(300);
            if (controller.get_digital(DIGITAL_L1)) {
                can_intake = false;
                intake_motor.move(0);
                pros::delay(350);
                can_intake = true;
            }
        }
        
        pros::delay(10);
    }

    std::cout << "blue: " << optical_sensor.get_rgb().blue;
    std::cout << " green: " << optical_sensor.get_rgb().green;
    std::cout << " red: " << optical_sensor.get_rgb().red << std::endl;
}

void initialize() {
    left_encoder.reset_position();
    left_encoder.set_data_rate(5);
    back_encoder.reset_position();
    back_encoder.set_data_rate(5);
    //inertial_sensor.reset();
    inertial_sensor.set_heading(0);
    inertial_sensor.set_data_rate(5);

    left_drive_motors.tare_position_all();
    right_drive_motors.tare_position_all();

    pros::Task odom_task(track_robot);
    //pros::Task colour_task(check_colour);

    lb_motors.set_brake_mode(pros::MotorBrake::hold);
}

void disabled() {}

// after init when in comp
void competition_initialize() {}


// INCREASE THREASHOLDS, MERGE CODE, CRY!

PID armpid = PID(1.50, 0.2, 0.15, 50, 5);

bool within_tolerance(double current, double target, double tolerance) {
    if (current - tolerance < target && current + tolerance > target) return true;
    return false;
}


void opcontrol() {
    while (inertial_sensor.is_calibrating()) pros::delay(5);
    left_drive_motors.set_brake_mode(pros::MotorBrake::coast);
    right_drive_motors.set_brake_mode(pros::MotorBrake::coast);

    Drive drive;
    bool mogo_state = false;

    int lift_target_pos = 0;
    bool move_lift_down = false;

    bool claw_lifted = false;

    int count = 0;
    int last_unclamp = 0;
    int last_autoclamp = -100;
    bool can_autoclamp = true;

    const double lb_rest_pos = 360;
    const double lb_pickup_pos = 335;
    const double lb_score_pos = 232;
    const double lb_tolerance = 5;
    int lb_state = 0; // 0 = still, 1 = move to rest, 2 = move to pickup, 3 = move to score
    int lb_dir = 1;

    while (true) {
        drive.movement();

        if (controller.get_digital_new_press(DIGITAL_R1) && count >= last_autoclamp + 100) {
            mogo_state = !mogo_state;
            mogo_piston.set_value(mogo_state);
            if (!mogo_state) last_unclamp = count;
            controller.rumble("_");
            can_autoclamp = false;
        }
        if (mogo_distance.get_distance() < 30 && can_autoclamp && !controller.get_digital(DIGITAL_B)) {
            mogo_state = true;
            mogo_piston.set_value(mogo_state);
            controller.rumble("-");
            last_autoclamp = count;
        }
        if (mogo_distance.get_distance() > 40 && count > last_unclamp + 200) can_autoclamp = true;

        if (controller.get_digital(DIGITAL_L1) && can_intake)
            intake_motor.move(127);
        else if (controller.get_digital(DIGITAL_L2)
                && !reversing) intake_motor.move(-127);
        else if (!reversing) intake_motor.move(0);

        if (controller.get_digital(DIGITAL_RIGHT)) doinker_piston.set_value(true);
        else doinker_piston.set_value(false);

        double lb_pos = lb_rotation_sensor.get_position() / 100.0;
        while (lb_pos < 180) lb_pos += 360;

        if (controller.get_digital_new_press(DIGITAL_R2)) {
            if (within_tolerance(lb_pos, lb_pickup_pos, lb_tolerance)) {
                lb_state = 1;
                lb_dir = 1;
            } else if (lb_state == 2 && lb_dir == 1) {
                lb_state = 1;
            } else {
                lb_state = 2;
                if (lb_pos < lb_pickup_pos) lb_dir = 1;
                else lb_dir = -1;
            }
        }
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            if (within_tolerance(lb_pos, lb_pickup_pos, lb_tolerance)) {
                lb_state = 3;
                lb_dir = -1;
            }
        }
                
        switch (lb_state) {
            case 0:
                break;
            case 1:
                if (within_tolerance(lb_pos, lb_rest_pos, lb_tolerance)) {
                    lb_motors.move_velocity(0);
                    lb_state = 0;
                } else {
                    lb_motors.move(lb_dir * arm_pid(armpid, lb_pos, lb_rest_pos));
                }
                break;
            case 2:
                if (within_tolerance(lb_pos, lb_pickup_pos, lb_tolerance)) {
                    lb_motors.move_velocity(0);
                    lb_state = 0;
                } else {
                    lb_motors.move(lb_dir * arm_pid(armpid, lb_pos, lb_pickup_pos));
                }
                break;
            case 3:
                if (within_tolerance(lb_pos, lb_score_pos, lb_tolerance)) {
                    lb_motors.move_velocity(0);
                    lb_state = 0;
                } else {
                    lb_motors.move(lb_dir * arm_pid(armpid, lb_pos, lb_score_pos));
                }
                break;
        }

        if (count % 500 == 0) {
            std::cout << "x: " << position.x << ", y: " << position.y << ", theta: " << position.theta << std::endl;
            std::cout << "lb pos: " << lb_pos << std::endl;
        }
        count++;

        pros::delay(5);
    }
}
