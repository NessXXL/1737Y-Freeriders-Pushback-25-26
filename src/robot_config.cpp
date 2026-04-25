#include "main.h" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "robot_con.hpp"

// master
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup left_mg({-1, -18, -12}, pros::MotorGearset::blue);  
pros::MotorGroup right_mg({13, 14, 15}, pros::MotorGearset::blue);

//intake
pros::Motor intake1(17);
pros::Motor intake2(-4);
pros::Motor intake3(21);


//Sensors
pros::Imu imu(3);
pros::Rotation horizontal_sensor(-19);
pros::Rotation vertical_sensor(16);
pros::Optical OpticalSensor(20);

//Pnumatics
pros::adi::Pneumatics loader('E',false);

pros::adi::Pneumatics wing('G', true);

pros::adi::Pneumatics intakeLOW('A', false);

pros::adi::Pneumatics wingScore('C', false);

pros::adi::Pneumatics doublep('D', false);


namespace Sensor{

pros::Distance d_left(9);

pros::Distance d_right(10);

pros::Distance d_front(8);

}