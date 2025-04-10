#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"

pros::MotorGroup left_motors({-20, -19, -7}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({15, 11, 1}, pros::MotorGearset::blue);

pros::Motor intake(3); // all in one motor

pros::MotorGroup ladybrown({14, -10});

pros::adi::DigitalOut solenoidExtend('H');

pros::Imu imu(12                                        );

pros::Optical coloursensor(18);

void forwards(double time) { // move forwards
  left_motors.move_voltage(12000);
  right_motors.move_voltage(12000);
  pros::delay(time);
  left_motors.move_voltage(0);
  right_motors.move_voltage(0);
}

void backwards(double time) { // move backwards
  left_motors.move_voltage(-12000);
  right_motors.move_voltage(-12000);
  pros::delay(time);
  left_motors.move_voltage(0);
  right_motors.move_voltage(0);
}

void slowbackwards(double time) { // move backwards
  left_motors.move_voltage(-6000);
  right_motors.move_voltage(-6000);
  pros::delay(time);
  left_motors.move_voltage(0);
  right_motors.move_voltage(0);
}

void left(double time) { // turn left
  left_motors.move_voltage(-12000);
  right_motors.move_voltage(12000);
  pros::delay(time);
  left_motors.move_voltage(0);
  right_motors.move_voltage(0);
}

void right(double time) { // turn right
  left_motors.move_voltage(12000);
  right_motors.move_voltage(-12000);
  pros::delay(time);
  left_motors.move_voltage(0);
  right_motors.move_voltage(0);
}

void openmogo(double time) { // opens mogo mech
  solenoidExtend.set_value(true);
  pros::delay(time);
  solenoidExtend.set_value(false);
}

void closemogo(double time) { // clses mogo mech
  solenoidExtend.set_value(true);
  pros::delay(time);
  solenoidExtend.set_value(false);
}

lemlib::Drivetrain drivetrain(&left_motors, &right_motors,
                              13.5, // change track width once built
                              lemlib::Omniwheel::NEW_275, 450,
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr,
                            & imu // inertial sensor
);

lemlib::ExpoDriveCurve throttle_curve( 3, 10, 1.040 );
lemlib::ExpoDriveCurve steer_curve( 3, 10, 1.040 );

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, & throttle_curve, & steer_curve            // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// initialize function. Runs on program startup

void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  pros::lcd::print(1,"hi");
  pros::delay(20);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

// void autonomous() {

//   int whatauton = 1;
//   switch (whatauton) {
//   case 1:
//   break;
//   }  }

void autonomous(){
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 100000);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
#define DIGITAL_SENSOR_PORT 'H'
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    double left_speed =
        controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) +
        controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    double right_speed =
        controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) -
        controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    left_motors.move_voltage(left_speed * (12000 / 127));
    right_motors.move_voltage(right_speed * (12000 / 127));

    // Uses the R buttons to control the wheels on intake


     // Uses the L button to control hooks on intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move_voltage(12000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move_voltage(-12000);
    } else {
      intake.move_voltage(0);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      solenoidExtend.set_value(true);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      solenoidExtend.set_value(false);
    } else {
      solenoidExtend.set_value(false);
    }

    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
       if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      ladybrown.move_voltage(-12000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      ladybrown.move_voltage(12000);
    } else {
      ladybrown.move_velocity(0);
      ladybrown.brake();
    
    }
    pros::delay(20);

    /*pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with
    forwards ports 1 & 3 and reversed port 2 pros::MotorGroup right_mg({-4, 5,
    -6});  // Creates a motor group with forwards port 5 and reversed ports 4 &
    6


    while (true) {
            pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() &
    LCD_BTN_LEFT) >> 2, (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                             (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
    // Prints status of the emulated screen LCDs

            // Arcade control scheme
            int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount
    forward/backward from left joystick int turn =
    master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right
    joystick left_mg.move(dir - turn);                      // Sets left motor
    voltage right_mg.move(dir + turn);                     // Sets right motor
    voltage pros::delay(20);                               // Run for 20 ms then
    update
    } */
  }
}