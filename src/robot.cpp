#include "robot.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive_motors({11, 12, 13});
pros::MotorGroup right_drive_motors({18, 19, 20});

pros::adi::DigitalOut mogo_piston(1);
pros::adi::DigitalOut intake_lift_piston(2);
pros::adi::DigitalOut claw_lift_piston(3);

pros::Imu inertial_sensor(14);
pros::Rotation left_encoder(16);
pros::Rotation back_encoder(15);

pros::Motor intake_motor(10);

pros::Optical optical_sensor(1);
pros::Distance distance_sensor(9);
