#include "robot.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive_motors({17, 18, 20}, pros::v5::MotorGears::green);
pros::MotorGroup right_drive_motors({11, 12, 13}, pros::v5::MotorGears::green);

pros::adi::DigitalOut mogo_piston(8); // H
pros::adi::DigitalOut doinker_piston(7);

pros::Imu inertial_sensor(2);
pros::Rotation left_encoder(16);
pros::Rotation back_encoder(15);

pros::Motor intake_motor(14, pros::MotorGearset::blue);

pros::MotorGroup lb_motors({-1, 10});
pros::Rotation lb_rotation_sensor(8);

pros::Optical optical_sensor(5);
pros::Distance mogo_distance(3);

Position position;