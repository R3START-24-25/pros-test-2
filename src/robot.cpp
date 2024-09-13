#include "robot.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive_motors({11, 12, 13});
pros::MotorGroup right_drive_motors({18, 19, 20});

pros::adi::DigitalOut mogo_piston(1);

pros::Imu inertial_sensor(17);
pros::Rotation left_encoder(16);
pros::Rotation back_encoder(15);
