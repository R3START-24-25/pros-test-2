#include "main.h"
#include "pros/abstract_motor.hpp"
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

class PID {
    public: float kP;
            float kI;
            float kD;
            float last_delta_position;
            float integral;
            float integral_threshold;
            int dt;

            PID(float kP, float kI, float kD, float integral_threshold, int dt) {
                this->kP = kP;
                this->kI = kI;
                this->kD = kD;
                this->integral_threshold = integral_threshold;
                this->dt = dt;
            }
};

float move_pid(PID pid, char axis, float target) {
    float current_position = axis == 'x' ? position.x : axis == 'y' ? position.y : position.theta;
    float delta_position = target - current_position;
    if (axis == 't') {
        if (delta_position > 180) delta_position -= 360;
        else if (delta_position < -180) delta_position += 360;
    }
    
    pid.integral = fabs(delta_position) > 0.05 && delta_position < pid.integral_threshold
        ? pid.integral + delta_position
        : 0;
    
    float derivative = delta_position - pid.last_delta_position;

    //if (delta_position > 0 && pid.last_delta_position < 0 || delta_position < 0 && pid.last_delta_position > 0) return 0;
    pid.last_delta_position = delta_position;

    float p_i_d = (delta_position * pid.kP) + (pid.integral * pid.kI) + (derivative * pid.kD);
    if (fabs(p_i_d) < 0.1 && fabs(delta_position) < 0.2) return 0;
    return p_i_d;
}

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
        float left_reading = left_encoder.get_position() / 100.0; // convert to degrees
        float back_reading = back_encoder.get_position() / 100.0;
        if (imu_reading > 360) imu_reading = 0;

        float delta_theta = imu_reading - last_imu_reading;
        if (delta_theta > 180) delta_theta -= 360; // assume wraparoud
        else if (delta_theta < -180) delta_theta += 360;
        float delta_left_reading = left_reading - last_left_reading;
        float delta_back_reading = back_reading - last_back_reading;

        float delta_dist_left = M_PI*wheel_diameter * (delta_left_reading/360);
        float delta_dist_back = M_PI*wheel_diameter * (delta_back_reading/360);

        position.theta = imu_reading;

        float delta_local_offset[2];
        if (fabs(delta_theta) < 0.0001) {
            delta_local_offset[0] = delta_dist_back;
            delta_local_offset[1] = delta_dist_left;
        }
        else {
            const float constant = 2*sinf((delta_theta * M_PI) / 360);
            delta_local_offset[0] = constant * ( ( delta_dist_back / ((delta_theta * M_PI)/180) ) + back_offset );
            delta_local_offset[1] = constant * ( ( delta_dist_left / ((delta_theta * M_PI)/180) ) - left_offset );
        }

        float avg_theta = last_imu_reading + (delta_theta / 2);
        if (avg_theta > 360) avg_theta -= 360; // wraparound
        float avg_theta_rad = avg_theta * (M_PI / 180.0);

        float delta_global_offset[2] = { // cartesian
            delta_local_offset[0] * cosf(-1*avg_theta_rad) - delta_local_offset[1] * sinf(-1*avg_theta_rad),
            delta_local_offset[0] * sinf(-1*avg_theta_rad) + delta_local_offset[1] * cosf(-1*avg_theta_rad)
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

void turn_to(float heading, PID pid, float speed) {
    float delta_theta = heading - position.theta;
    bool left = delta_theta > 180 ? false : true;
    int count = 0;

    while (true) {
        //float pid_out = move_pid(pid, 't', heading);
        //float velocity = pid_out;

        float velocity = left ? -70 : 70;

        left_drive_motors.move(speed * velocity);
        right_drive_motors.move(speed * velocity);

        if (fabs(velocity) < 0.1 || count == 1000) break;
        if (position.theta > heading - 2 && position.theta < heading + 2) break;

        count += pid.dt;

        pros::delay(pid.dt);
    }
    left_drive_motors.move(0);
    right_drive_motors.move(0);
}

void move_to(float magnitude, char axis, PID pid, float speed) {
    int count = 0;
    int multi = axis == 'x' ? -1 : 1;

    while (true) {
        float pid_out = move_pid(pid, axis, magnitude);
        left_drive_motors.move(speed * pid_out * -1 * multi);
        right_drive_motors.move(speed * pid_out * multi);

        float pos = axis == 'x' ? position.x : position.y;
        if (pid_out == 0 || count == 1000) break;
        count += pid.dt;

        pros::delay(pid.dt);
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

void autonomous() {
    PID xy_pid(1, 0, 0.25, 12, 5);
    PID turn_pid(0.6, 0, 0, 12, 5);

    left_drive_motors.set_brake_mode(pros::MotorBrake::hold);
    right_drive_motors.set_brake_mode(pros::MotorBrake::hold);

    while (inertial_sensor.is_calibrating()) pros::delay(5);

    mogo_piston.set_value(true);
    move_to(-21, 'y', xy_pid, 3.5);
    mogo_piston.set_value(false);
    // mogo picked up
    move_to(-29, 'y', xy_pid, 3.5);
    intake_motor.move(-127);
    turn_to(302, turn_pid, 0.7); // 270 degrees
    move_to(-24, 'x', xy_pid, 3.5);
    // ring picked up
    turn_to(330, turn_pid, 0.7); // 0 d
    move_to(-5, 'y', xy_pid, 3.5);
    turn_to(55, turn_pid, 0.7); // 90 d
    move_to(18, 'x', xy_pid, -3.5);
    mogo_piston.set_value(true);
}

void opcontrol() {
    left_drive_motors.set_brake_mode(pros::MotorBrake::coast);
    right_drive_motors.set_brake_mode(pros::MotorBrake::coast);

    Drive drive;
    bool mogo_state = false;
    int count = 0;

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
            if (lift_is_down) {
                lift_target_pos = 6000;
                lift_is_down = false;
            }
            else lift_target_pos = 9000;
            move_lift_down = false;
        }
        if (controller.get_digital_new_press(DIGITAL_DOWN)) {
            lift_target_pos = 3;
            move_lift_down = true;
            lift_is_down = true;
        }

        move_intake(lift_target_pos, move_lift_down);

        if (count % 500 == 0) std::cout << position.x << " " << position.y << " " << position.theta << "\n";
        count++;

        pros::delay(5);
    }
}
