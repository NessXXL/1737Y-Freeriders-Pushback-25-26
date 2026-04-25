#include "main.h"
#include "freeriders/api.hpp" // IWYU pragma: keep
#include "freeriders/chassis/chassis.hpp"
#include "freeriders_con.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot_con.hpp"
//#include "freeriders/pose.hpp"

    struct PoseSampleParams {
        double refX = std::numeric_limits<double>::quiet_NaN();
        double refY = std::numeric_limits<double>::quiet_NaN();
        double radiusIn = 8.0;
    };
    static bool poseWithinRadius(double x, double y, double refX, double refY, double radiusIn) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(refX) || !std::isfinite(refY)) return false;
        const double dx = x - refX;
        const double dy = y - refY;
        return (dx * dx + dy * dy) <= (radiusIn * radiusIn);
    }

    static void applyPoseFallback(double& x, double& y, double estimateX, double estimateY, const PoseSampleParams& params) {
        const double refX = std::isfinite(params.refX) ? params.refX : estimateX;
        const double refY = std::isfinite(params.refY) ? params.refY : estimateY;
        const double radiusIn = std::isfinite(params.radiusIn) ? params.radiusIn : 0.0;
        if (radiusIn <= 0.0 || !poseWithinRadius(x, y, refX, refY, radiusIn)) {
            x = refX;
            y = refY;
        }
    }

void resetWalls(bool useLeft = true, bool useRight = true, bool useFront = true, PoseSampleParams sampleParams = PoseSampleParams{}) {
        constexpr double field = 144.0;
        constexpr double halfField = field / 2.0;
        constexpr double WALL_0_X = halfField;
        constexpr double WALL_1_Y = halfField;
        constexpr double WALL_2_X = -halfField;
        constexpr double WALL_3_Y = -halfField;
        constexpr double ANGLE_TOLERANCE = 15.0 * (M_PI / 180.0);
        constexpr double pi = M_PI;
        constexpr double mmToIn = 1.0 / 25.4;


        
        constexpr double leftOffsetR = -5.7;
        constexpr double rightOffsetR = 4.5;
        constexpr double frontOffsetF = 6.5;
        constexpr double frontOffsetR = 0.0;
        // constexpr double leftOffsetR = -7.1;
        // constexpr double rightOffsetR = 7.1;
        // constexpr double frontOffsetF = 9.0;
        // constexpr double frontOffsetR = 0.0;

        enum class Axis { NONE, X, Y };
        struct Result {
            Axis axis;
            double axisPosition;
        };

        freeriders::Pose m_offset(0.0, 0.0, 0.0);
        double realReading = 0.0;

        auto getReading = [&](freeriders::Pose pose, Result& result, bool force) {
            double sensorAngle = pose.theta + m_offset.theta;
            double sensorOffsetX = m_offset.x * std::cos(pose.theta) - m_offset.y * std::sin(pose.theta);
            double sensorOffsetY = m_offset.x * std::sin(pose.theta) + m_offset.y * std::cos(pose.theta);
            double sensorX = pose.x + sensorOffsetX;
            double sensorY = pose.y + sensorOffsetY;

            double predictedReading;
            double angleError;
            int wall;

            if (angleError = std::abs(std::remainder(0 - sensorAngle, 2 * pi)); angleError < ANGLE_TOLERANCE) {
                predictedReading = (WALL_0_X - sensorX) / std::cos(angleError);
                wall = 0;
            } else if (angleError = std::abs(std::remainder(0.5 * pi - sensorAngle, 2 * pi)); angleError < ANGLE_TOLERANCE) {
                predictedReading = (WALL_1_Y - sensorY) / std::cos(angleError);
                wall = 1;
            } else if (angleError = std::abs(std::remainder(pi - sensorAngle, 2 * pi)); angleError < ANGLE_TOLERANCE) {
                predictedReading = (sensorX - WALL_2_X) / std::cos(angleError);
                wall = 2;
            } else if (angleError = std::abs(std::remainder(1.5 * pi - sensorAngle, 2 * pi)); angleError < ANGLE_TOLERANCE) {
                predictedReading = (sensorY - WALL_3_Y) / std::cos(angleError);
                wall = 3;
            } else {
                wall = -1;
            }

            if (wall == 0) {
                result.axis = Axis::X;
                result.axisPosition = (WALL_0_X - realReading * std::cos(angleError)) - sensorOffsetX;
            } else if (wall == 1) {
                result.axis = Axis::Y;
                result.axisPosition = (WALL_1_Y - realReading * std::cos(angleError)) - sensorOffsetY;
            } else if (wall == 2) {
                result.axis = Axis::X;
                result.axisPosition = (WALL_2_X + realReading * std::cos(angleError)) - sensorOffsetX;
            } else if (wall == 3) {
                result.axis = Axis::Y;
                result.axisPosition = (WALL_3_Y + realReading * std::cos(angleError)) - sensorOffsetY;
            }
        };

        freeriders::Pose pose = chassis.getPose(true, true);
        double heading = chassis.getPose().theta;
        const double estimatedX = pose.x;
        const double estimatedY = pose.y;

        Result leftRes = {Axis::NONE, 0.0};
        Result rightRes = {Axis::NONE, 0.0};
        Result frontRes = {Axis::NONE, 0.0};

        if (useLeft) {
            m_offset = freeriders::Pose(0.0, -leftOffsetR, M_PI_2);
            realReading = Sensor::d_left.get_distance() * mmToIn;
            getReading(pose, leftRes, false);
        }
        if (useRight) {
            m_offset = freeriders::Pose(0.0, -rightOffsetR, -M_PI_2);
            realReading = Sensor::d_right.get_distance() * mmToIn;
            getReading(pose, rightRes, false);
        }
        if (useFront) {
            m_offset = freeriders::Pose(frontOffsetF, -frontOffsetR, 0.0);
            realReading = Sensor::d_front.get_distance() * mmToIn;
            getReading(pose, frontRes, false);
        }

        double x = estimatedX;
        double y = estimatedY;
        double xSum = 0.0;
        double ySum = 0.0;
        int xCount = 0;
        int yCount = 0;

        auto accumulate = [&](const Result& res) {
            if (!std::isfinite(res.axisPosition)) return;
            if (res.axis == Axis::X) {
                xSum += res.axisPosition;
                xCount++;
            } else if (res.axis == Axis::Y) {
                ySum += res.axisPosition;
                yCount++;
            }
        };

        if (useLeft) accumulate(leftRes);
        if (useRight) accumulate(rightRes);
        if (useFront) accumulate(frontRes);

        if (xCount > 0) x = xSum / static_cast<double>(xCount);
        if (yCount > 0) y = ySum / static_cast<double>(yCount);

        applyPoseFallback(x, y, estimatedX, estimatedY, sampleParams);

        chassis.setPose(x, y, heading);

        printf("Pose -> X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, heading);
    }

void cdrift(float lV, float rV, int timeout, bool cst = true){
       (cst == true) ? (left_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST), right_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST)) : (left_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE), right_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE));
       left_mg.move(lV);
       right_mg.move(rV);
       pros::delay(timeout);
       left_mg.brake();
       right_mg.brake();
}

bool runOpticalSensor1 = true;
void OpticalSensorRED1() {
    while (true) {
        if(runOpticalSensor1 ){
            if ((OpticalSensor.get_hue() > 340 && OpticalSensor.get_hue() < 360 && OpticalSensor.get_proximity() <= 95) || (OpticalSensor.get_hue() > 0.01 && OpticalSensor.get_hue() < 1.1 && OpticalSensor.get_proximity() <= 95)) {
                intake3.move(127);
                intake2.move(-127);  
                
            } //else {
                //intconv.move_voltage(-12000);
            //}
        }
        pros::delay(20); 
    }
}  


void OpticalSensorBLUE1() {
    while (true) {
        if(runOpticalSensor1 ){
            if ( OpticalSensor.get_proximity() <= 95){
                intake3.move(0);
                
            } else {
                intake3.move_voltage(12000);
            }
        }
        pros::delay(20);
    }
}
    

void initialize() {
    // lv_init();
    // lv_img_set_src(slogo, &speed_on_brain);
    // lv_obj_set_pos(slogo, 160, 0);
    //chassis.calibrate();
    pros::lcd::initialize();
    imu.reset();
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    //wing.set_value(true);
    OpticalSensor.set_led_pwm(127);

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            freeriders::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });

       
}



/**
 * Runs while the robot is disabled
 */
void disabled() {}



/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); 
ASSET(skills1_txt);
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */


namespace Auton{
    void Skills(){ 

        chassis.setPose(-48, 6, 0);
        intake1.move(127);intake2.move(127);intake3.move(95);
        intakeLOW.set_value(true);
        chassis.moveToPoint(-46, 48, 1200, {.forwards = true, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1});
        chassis.turnToPoint(-67, 48, 900,{.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.waitUntilDone();
        loader.set_value(true);  
        chassis.moveToPoint(-69, 48, 1050,{.forwards = true, .maxSpeed = 62, .minSpeed = 0, .earlyExitRange = 1} );
        pros::delay(500);
        chassis.moveToPoint(-73, 46, 1050,{.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 1} );
        pros::delay(2200);
        chassis.moveToPoint(-57, 46, 1000,{.forwards = false, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(-30, 59, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(24, 62, 1000,{.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(46, 47.5, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.turnToPoint(67, 50, 900,{.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(26, 47, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        pros::delay(600);
        wingScore.set_value(true);
        pros::delay(2200);
        chassis.moveToPoint(73, 45.5, 1050,{.forwards = true, .maxSpeed = 63, .minSpeed = 0, .earlyExitRange = 1} );
        pros::delay(200);
        wingScore.set_value(false);
        pros::delay(500);
        chassis.moveToPoint(80, 46, 1050,{.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 1} );
        pros::delay(2200);
        //wingScore.set_value(false);
        chassis.moveToPoint(26, 46.9, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        pros::delay(800);
        wingScore.set_value(true);
        pros::delay(2200);
        chassis.moveToPoint(46, 46, 1000,{.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.waitUntilDone();
        loader.set_value(false);
        intakeLOW.set_value(true);
        chassis.turnToPoint(39, 0, 900,{.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1} );
        wingScore.set_value(false);
        chassis.moveToPoint(39, 2, 1000,{.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(27, -29, 1000,{.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.waitUntil(27);
        loader.set_value(true);
        chassis.waitUntilDone();
        pros::delay(600);
        chassis.turnToPoint(47, -50, 900,{.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(47, -54, 1000,{.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1} );
        loader.set_value(true);
        chassis.turnToPoint(69, -53, 900,{.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(73, -53, 1200,{.forwards = true, .maxSpeed = 62, .minSpeed = 0, .earlyExitRange = 1} );
        pros::delay(500);
        chassis.moveToPoint(77, -53, 1050,{.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 1} );
        pros::delay(2000);
        resetWalls(false, true, false, {60, -49, 5});
        pros::delay(500);
        //pros::delay(10000000000000000);
        chassis.moveToPoint(57, -48, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(36, -62, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        loader.set_value(false);
        chassis.moveToPoint(-23, -64.5, 1200,{.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(-47, -49, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.turnToPoint(-66, -48, 900,{.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.moveToPoint(-24, -49.7, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        chassis.waitUntilDone();
        wingScore.set_value(true);
        pros::delay(2500);
        loader.set_value(true);
        chassis.moveToPoint(-75, -48.8, 1000,{.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 1} );
        wingScore.set_value(false);
        pros::delay(500);
        chassis.moveToPoint(-79, -48.8, 1000,{.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 1} );
        pros::delay(2050);
        chassis.moveToPoint(-24, -49, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
        pros::delay(700);
        wingScore.set_value(true);
        pros::delay(2200);
        chassis.moveToPoint(-46, -49.3, 1000,{.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
        wingScore.set_value(false);
        loader.set_value(false);
        chassis.waitUntilDone();
        chassis.moveToPoint(-25, -23, 1000, {.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1});
        chassis.moveToPoint(-26, 26, 1500, {.forwards = true, .maxSpeed = 120, .minSpeed = 0, .earlyExitRange = 1});
        chassis.waitUntil(48.5);
        loader.set_value(true);
        pros::delay(400);
        chassis.waitUntilDone();
        //-12 13
        chassis.moveToPoint(-11, 10, 1000, {.forwards = false, .maxSpeed = 65, .minSpeed = 0, .earlyExitRange = 1});
        pros::delay(750);
        intake1.move(127);intake2.move(127);intake3.move(-85);
        pros::delay(2000);
        chassis.moveToPoint(-24, 26, 1000, {.forwards = true, .maxSpeed = 120, .minSpeed = 0, .earlyExitRange = 1});
        loader.set_value(false);
        intake1.move(127);intake2.move(127);intake3.move(78);
        chassis.turnToPoint(-63, 25, 1000, {.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1});
        chassis.moveToPoint(-63, 25, 1000, {.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1});
        chassis.turnToPoint(-46, 58, 1000, {.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1});
        chassis.moveToPoint(-65, -18.2, 1600, {.forwards = false, .maxSpeed = 127, .minSpeed = 120, .earlyExitRange = 1});
        
    }
    namespace BLUE{
        namespace Qual{
            void blueSolo(){
                intake1.move(127);intake2.move(127);intake3.move(127);
                loader.set_value(true);
                chassis.setPose(60, -50, 90);
                pros::delay(500);
                chassis.moveToPoint(57, -48, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
                chassis.moveToPoint(36, -62, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
                loader.set_value(false);
                chassis.moveToPoint(-23, -64.5, 1200,{.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
                chassis.moveToPoint(-47, -49, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
                chassis.turnToPoint(-66, -48, 900,{.forwards = true, .maxSpeed = 90, .minSpeed = 0, .earlyExitRange = 1} );
                chassis.moveToPoint(-24, -49.7, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
                chassis.waitUntilDone();
                wingScore.set_value(true);
                pros::delay(2200);
                loader.set_value(true);
                chassis.moveToPoint(-75, -49, 1000,{.forwards = true, .maxSpeed = 70, .minSpeed = 0, .earlyExitRange = 1} );
                wingScore.set_value(false);
                pros::delay(500);
                chassis.moveToPoint(-79, -49, 1000,{.forwards = true, .maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 1} );
                pros::delay(2050);
                chassis.moveToPoint(-24, -49, 1000,{.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1} );
                pros::delay(700);
                wingScore.set_value(true);
                pros::delay(1900);
                chassis.moveToPoint(-46, -49.3, 1000,{.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
                wingScore.set_value(false);
                chassis.waitUntilDone();
                chassis.turnToPoint(-30, -65, 900,{.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
                loader.set_value(false);
                chassis.moveToPose(-67, -10, 180, 1200,{.forwards = false, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1} );
                chassis.moveToPoint(-68, 37.5, 1000,{.forwards = false, .maxSpeed = 127, .minSpeed = 60, .earlyExitRange = 1} );
                
            }

            void blueAWPUni(){

            }
                
                
        }
        
        namespace Finals{
            void low(){
                chassis.setPose(-47.5, -14, 180);
                intake1.move(127);intake2.move(127);intake3.move(127);
                chassis.moveToPoint(-48, -47.8, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1});
                chassis.turnToHeading(270, 700,{.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 120, .minSpeed = 20, .earlyExitRange = 1} );
                loader.set_value(true);
                intakeLOW.set_value(true);
                chassis.moveToPoint(-67, -48.7, 1000, {.forwards = true, .maxSpeed = 67, .minSpeed = 0, .earlyExitRange = 1});
                pros::delay(1200);
                chassis.moveToPoint(-25, -49, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1});
                chassis.waitUntil(28);
                wingScore.set_value(true);
                pros::delay(1500);
                wingScore.set_value(false);
                cdrift(50, 50, 300);
                cdrift(50, 15, 400);
                loader.set_value(false);
                chassis.moveToPoint(-24, -22.5, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1});
                chassis.waitUntil(29);
                loader.set_value(true);
                pros::delay(400);
                cdrift(10, 10, 400);
                loader.set_value(false);
                chassis.turnToPoint(-10, -8, 900, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1});
                chassis.moveToPoint(-10, -8, 900, {.forwards = true, .maxSpeed = 60, .minSpeed = 0, .earlyExitRange = 1});
                pros::delay(300);
                intake1.move(-80);intake2.move(-127);intake3.move(-127);
                pros::delay(1200);
                intake1.move(-120);intake2.move(-127);intake3.move(-127);
                cdrift(-15, -15, 1000);
            }
        }
    }
        

    

    namespace RED{
        namespace Qual{
            void redSolo(){
                chassis.setPose(-47.5, -14, 180);
                intake1.move(127);intake2.move(127);intake3.move(127);
                chassis.moveToPoint(-48, -47.8, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1});
                chassis.turnToHeading(270, 700,{.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 120, .minSpeed = 20, .earlyExitRange = 1} );
                loader.set_value(true);
                intakeLOW.set_value(true);
                chassis.moveToPoint(-67, -48.7, 1000, {.forwards = true, .maxSpeed = 66, .minSpeed = 0, .earlyExitRange = 1});
                pros::delay(300);
                chassis.moveToPoint(-25, -49, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 0, .earlyExitRange = 1});
                chassis.waitUntil(28);
                wingScore.set_value(true);
                pros::delay(1180);
                wingScore.set_value(false);
                cdrift(50, 50, 300);
                cdrift(50, 15, 400);
                loader.set_value(false);
                chassis.moveToPoint(-25, -28, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1});
                chassis.moveToPoint(-24, 25.35, 1500, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1});
                chassis.waitUntil(48);
                loader.set_value(true);
                pros::delay(400);
                chassis.waitUntilDone();
                chassis.moveToPoint(-12.5, 13.2, 1000, {.forwards = false, .maxSpeed = 65, .minSpeed = 0, .earlyExitRange = 1});
                pros::delay(750);
                intake1.move(127);intake2.move(127);intake3.move(-117);
                pros::delay(1400);
                chassis.moveToPoint(-48, 47.8, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed = 0, .earlyExitRange = 1});
                intake1.move(15);intake2.move(15);intake3.move(-70);
                chassis.turnToHeading(270, 700,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 120, .minSpeed = 20, .earlyExitRange = 1} );
                loader.set_value(true);
                intake1.move(127);intake2.move(127);intake3.move(127);
                chassis.moveToPoint(-68, 51, 1000, {.forwards = true, .maxSpeed = 85, .minSpeed = 0, .earlyExitRange = 1});
                pros::delay(400);
                chassis.moveToPoint(-25, 51.8, 1000, {.forwards = false, .maxSpeed = 120, .minSpeed = 0, .earlyExitRange = 1});
                chassis.waitUntil(25);
                wingScore.set_value(true);
            }

            void redAWPUni(){
            
            }
        }

        namespace Finals{
            void redMinus(){
            }
        }
    }

}



void autonomous() {
    // chassis.setPose(-24, -48, 0);
    // chassis.turnToHeading(270, 800, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 1});
    // pros::delay(2000);
    // reset1();

    //Auton::BLUE::Qual::blueSolo();
    Auton::Skills();
    Auton::RED::Qual::redSolo();
    //Auton::BLUE::Finals::low();
    // resetWalls(true, false, true);
    //cdrift(20, 20, 500);
    //chassis.setPose(0, 0, 0);
    //chassis.moveToPoint(0, 24, 1000, {.maxSpeed = 90});
    //chassis.turnToHeading(90, 1000);
}

bool descoreState = false; 
bool colorState = false;

   namespace Driver{
    bool solenoidState = false;  
    bool lastButtonState = false; 
    bool solenoidState1 = false;  
    bool lastButtonState1 = false;
    bool solenoidState2 = false;  
    bool lastButtonState2 = false;
    bool solenoidState3 = false;  
    bool lastButtonState3 = false;  

    void intakeS(){
        while(1){
            bool currentButtonState3 = controller.get_digital(DIGITAL_B);
            if (currentButtonState3 && !lastButtonState3) {
                solenoidState3 = !solenoidState3; 
                intakeLOW.set_value(solenoidState3);
            }
            lastButtonState3 = currentButtonState3;
            pros::delay(10);
        }
    }

    void arcade1(){
        while(1){
            int leftY = controller.get_analog(ANALOG_LEFT_Y);
            int rightX = controller.get_analog(ANALOG_RIGHT_X);
            chassis.arcade(leftY, rightX);
            pros::delay(10);

            //TANK
            // int leftY = controller.get_analog(ANALOG_LEFT_Y);
            // int rightY = controller.get_analog(ANALOG_RIGHT_Y);
            // chassis.tank(leftY, rightY);
            // pros::delay(10);

            //ONE STICK
            // int leftY = controller.get_analog(ANALOG_LEFT_Y);
            // int rightX = controller.get_analog(ANALOG_LEFT_X);
            // chassis.arcade(leftY, rightX);
            // pros::delay(10);
        }
    }

    void intake0(){
        while(1){
            if(controller.get_digital(DIGITAL_R1))
            {
                intake1.move_voltage(12000);
                intake2.move_voltage(12000);
                intake3.move_voltage(12000);
                
            }
            else if(controller.get_digital(DIGITAL_L1))
            {
                intake1.move_voltage(12000);
                intake2.move_voltage(12000);
                intake3.move_voltage(12000);
                
            }
            // else if(controller.get_digital(DIGITAL_R1))
            // {
            //     intake.move_voltage(12000);
            //     intake.move_voltage(12000);
            //     intake.move_voltage(-12000);
                
            // }
            else if(controller.get_digital(DIGITAL_L2))
            {
                intake1.move_voltage(12000);
                intake2.move_voltage(12000);
                intake3.move_voltage(-12000);
                
            }
            else if(controller.get_digital(DIGITAL_R2))
            {
                intake1.move_voltage(-12000);
                intake2.move_voltage(-12000);
                intake3.move_voltage(-12000);
                
            }
            else if(controller.get_digital(DIGITAL_X))
            {
                intake1.move_voltage(-3000);
                intake2.move_voltage(-3000);
                intake3.move_voltage(-3000);
                
            }
            else{
                intake1.move_voltage(0);
                intake2.move_voltage(0);
                intake3.move_voltage(0);
                
            }
            pros::delay(10);
        }
    }
    

    // void wing1(){
    //     while(1){
    //         bool currentButtonState = controller.get_digital(DIGITAL_L1);
    //         if (currentButtonState && !lastButtonState) {
    //             solenoidState = !solenoidState;
    //             wingScore.set_value(solenoidState);
    //         }

            
    //         lastButtonState = currentButtonState;
    //         pros::delay(10);
    //     }
    // }

    void wing1(){
        while(1){
            if(controller.get_digital(DIGITAL_L1)){
                wingScore.set_value(true);
            }
            else{
                wingScore.set_value(false);
            }
            pros::delay(10);
        }
    }

    void loader1(){
        while(1){
            bool currentButtonState1 = controller.get_digital(DIGITAL_RIGHT);
            if (currentButtonState1 && !lastButtonState1) {
                solenoidState1 = !solenoidState1; 
                loader.set_value(solenoidState1);
            }
            lastButtonState1 = currentButtonState1;
            pros::delay(10);
        }
    }

    void doublePark1(){
        while(1){
            bool currentButtonState2 = controller.get_digital(DIGITAL_Y);
            if (currentButtonState2 && !lastButtonState2) {
                solenoidState2 = !solenoidState2; 
                intakeLOW.set_value(solenoidState2);
            }
            lastButtonState2 = currentButtonState2;
            pros::delay(10);
        }
    }

    
}

void opcontrol() {

    

    // Controller master(E_CONTROLLER_MASTER);
    pros::Task arcade_task(Driver::arcade1);
    pros::Task intconv_task(Driver::intake0);
    pros::Task solenoid_task(Driver::wing1);
    pros::Task shovel_task(Driver::doublePark1);
    pros::Task loader_task(Driver::loader1);
    pros::Task int_task(Driver::intakeS);
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    //intake1.move(127);intake2.move(127);intake3.move(127);
    while (true) {
        pros::delay(10);
    }

    autonomous();
}