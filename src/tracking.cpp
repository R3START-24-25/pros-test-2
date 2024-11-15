#include "robot.hpp"
#include <cmath>

void track_robot() {
    const float left_offset = -0.591;
    const float back_offset = 0;
    const float wheel_diameter = 2; // inches
    //const float temp_wheel_diameter = 2.0625; // inches

    float last_imu_reading  = 0;
    float last_left_reading = 0;
    float last_back_reading = 0;

    position.x = 0;
    position.y = 0;
    position.theta = 0;

    while (inertial_sensor.is_calibrating()) pros::c::delay(10);

	while (true) {
        float imu_reading = inertial_sensor.get_heading();
        if (imu_reading > 360) imu_reading = 0;
        float left_reading = left_encoder.get_position() / 100.0; // convert to degrees
        float back_reading = back_encoder.get_position() / 100.0;

        float delta_theta = imu_reading - last_imu_reading;

        if (delta_theta > 180) delta_theta -= 360; // assume wraparoud
        else if (delta_theta < -180) delta_theta += 360;
        float delta_left_reading = left_reading - last_left_reading;
        float delta_back_reading = back_reading - last_back_reading;

        float delta_dist_left = M_PI*wheel_diameter * (delta_left_reading/360);
        float delta_dist_back = M_PI*wheel_diameter * (delta_back_reading/360);

        position.theta = imu_reading;

        float delta_local_offset[2];
        if (fabs(delta_theta) < 0.001) {
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
