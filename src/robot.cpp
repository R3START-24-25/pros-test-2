#include "robot.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive_motors({11, 12, 13}, pros::v5::MotorGears::green);
pros::MotorGroup right_drive_motors({1, 2, 3}, pros::v5::MotorGears::green);

pros::adi::DigitalOut mogo_piston(1);
pros::adi::DigitalOut doinker_piston(2);

pros::Imu inertial_sensor(5);
pros::Rotation left_encoder(15);
pros::Rotation back_encoder(14);

pros::Motor intake_motor(4);

Position position;
