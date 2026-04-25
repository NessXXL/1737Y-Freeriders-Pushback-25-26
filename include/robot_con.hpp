#pragma once

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "main.h" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/optical.hpp"

extern pros::Controller controller;

extern pros::MotorGroup left_mg;

extern pros::MotorGroup right_mg;

extern pros::Motor intake1;

extern pros::Motor intake2;

extern pros::Motor intake3;

extern pros::adi::Pneumatics wing;
extern pros::adi::Pneumatics wingScore;
extern pros::adi::Pneumatics doublep;
extern pros::adi::Pneumatics intakeLOW;
extern pros::adi::Pneumatics loader;

namespace Sensor{ 
extern pros::Distance d_left;
extern pros::Distance d_right;
extern pros::Distance d_front;
}

extern pros::Imu imu;

extern pros::Optical OpticalSensor;

extern pros::Rotation vertical_sensor;
extern pros::Rotation horizontal_sensor;