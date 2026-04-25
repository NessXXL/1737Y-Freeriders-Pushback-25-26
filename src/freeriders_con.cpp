#include "freeriders/chassis/chassis.hpp"
#include "freeriders/chassis/trackingWheel.hpp"
#include "main.h" // IWYU pragma: keep
#include "robot_con.hpp"


freeriders::TrackingWheel horizontal(&horizontal_sensor, freeriders::Omniwheel::NEW_2, 3.75);
freeriders::TrackingWheel vertical(&vertical_sensor, freeriders::Omniwheel::NEW_2, 0.75);

// drivetrain settingss
freeriders::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor groups
                              10.83, // 10   inch track width
                              freeriders::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion master
freeriders::ControllerSettings linearController(6.2, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3.1, // derivative gain (kD)
                                            0, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
freeriders::ControllerSettings angularController(1.95,// proportional gain (kP)
                                             0, // integral gain (kI)
                                             15.1, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);


// sensors for odometry
freeriders::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizont al tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
freeriders::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
freeriders::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.021 // expo curve gain
);

// create the chassis
freeriders::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);