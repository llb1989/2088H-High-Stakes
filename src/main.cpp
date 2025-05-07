#include "main.h"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include <chrono>
#include <string>
#include <sys/_intsup.h>

/* ^ can move other includes to main.h ^ */

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-20, -19, -7}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({15, 11, 1}, pros::MotorGearset::blue);

pros::Motor intake(3); 

pros::MotorGroup ladybrown({14, -10});

pros::adi::DigitalOut solenoidExtend('H');
pros::adi::DigitalOut doinker('E');

pros::Rotation rotationsensor(4);
pros:: Imu imu (17);
pros::Optical coloursensor (13);

bool doinkerToggle = false; 
/* ^^ why 9+ errors? idk. ^^ */

/* colour sort */
int intakespeed = -12000;
bool isRed = true;

void colorsort() {
  pros::Task([]{
      while (true) {
        if ((isRed == false && (coloursensor.get_hue() > 010 && coloursensor.get_hue() < 030)) ||
            (isRed == true && coloursensor.get_hue() > 150 && coloursensor.get_hue() < 220)) 
        {
          
          pros::delay(58);
          //intake.brake();
          intakespeed = 0;
          intake.move_voltage(0); 
          
          pros::delay(280);
          intake.move_voltage(-12000);
          intakespeed = -12000;
         

      } else if ((isRed == false && (!coloursensor.get_hue() > 010 && !coloursensor.get_hue() < 030)) ||
                 (isRed == true  && (!coloursensor.get_hue() > 150 && !coloursensor.get_hue() < 230))){

          intakespeed = -12000;
          intake.move_voltage(-12000);
          

      }    
         pros::delay(90);
  }
});
}

/* Hard code functions */
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

void intakein(double time) { //intakes
  intake.move_voltage(-12000);
  pros::delay(time);
  intake.move_voltage(0);
}

void intakeout(double time) { //intakes
  intake.move_voltage(12000);
  pros::delay(time);
  intake.move_voltage(0);
}

void moveladybrown (double time, double voltage) {
  ladybrown.move(voltage);
  pros::delay(time);
  intake.move_voltage(0);
}

void intakestop (double time) {
  intake.brake();
  pros::delay(time);
  intake.move_voltage(0);
}

/*Declaring Lemlib sensors and drivetrain*/
lemlib::Drivetrain drivetrain(&left_motors, &right_motors,
                              13.5, // change track width once built
                              lemlib::Omniwheel::NEW_275, 450,
                              2
);

// odometry settings
lemlib::OdomSensors sensors(nullptr /* could use motor encoders?*/, nullptr, nullptr, nullptr,
                            & imu // inertial sensor
);

/*ask angela abt these*/
lemlib::ExpoDriveCurve throttle_curve( 3, 10, 1.040 );
lemlib::ExpoDriveCurve steer_curve( 3, 10, 1.040 );

// lateral control
lemlib::ControllerSettings lateral_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              2, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular control
lemlib::ControllerSettings angular_controller(2.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


/* creating chassis */
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors            // odometry sensors
);


/* Ladybug pid control */
static const int states = 3; // how many positions for the lb to be in
int wall_stake_positions[states] = {0, -32, -150}; // angle of lb
int current_state = 0; // starts at angle 0 always (whatever is initial lb position)
int target = wall_stake_positions[current_state];

void next_state(){ // forwards
    current_state++; // add one = move to next state
    if(current_state >= states){
        current_state = 0;
    } 
    target = wall_stake_positions[current_state]; // updates target
}

void prev_state(){ // backwards
    current_state--; // subtracr one = go to past state
    if(current_state < 0){
        current_state = states - 1;
    }
    target = wall_stake_positions[current_state]; // updates targer
}

    double kp = 2.5;
    double kd = 5.8;
    double error = 0;
    double previous_error = 0;
    double derivative = 0;

void power_wall_stake(){ 
    error = target - (rotationsensor.get_position() / 100); // ger rotation sensor angle
    derivative = error - previous_error;
    previous_error = error;

    double velocity = (kp * error) + (kd * derivative);
    ladybrown.move(velocity); // moves ladybug
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// initialize function. Runs on program startup

void initialize() {
  pros::lcd::initialize(); // initialize brain screen (doesn't work)
  chassis.calibrate(); // calibrates chassis/sensors
  ladybrown.set_brake_mode(pros::MotorBrake::hold); // stop lb from being pushed 
  rotationsensor.reset_position(); // CURRENT POSITION WILL BE ZERO
  coloursensor.set_integration_time(1);
  pros::Task screen_task([&]() {
      while (true) {
          
          //master.print(0, 0, "angle %d", imu.get_rotation());
          //master.print(0, 0, "y: %.2f, x: %.2f", chassis.getPose().y, chassis.getPose().x); // y value works, x is questionable
          master.print(0, 0,"Y: %.2f", chassis.getPose().y);
          /* 
          DOESN'T WORK?!?!?!?!?
          pros::lcd::print(0, "X: %.2f", chassis.getPose().x); // x position
          pros::lcd::print(1, "Y: %.2f", chassis.getPose().y); // y position
          pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta); // imu/angle of robot
          */
          pros::delay(10);
      }
  });
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
void competition_initialize() {


}

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

void autonomous(){
  coloursensor.set_integration_time(1);
  coloursensor.set_led_pwm(50);
  colorsort();
  chassis.moveToPoint(0, 0, 10); // resets chassis position
 
  int autoselect = 12; // change auton 
  switch (autoselect) {
    
    case 1: // blue ring rush side (one ring)
 
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -30, 1500, {.forwards = false}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(false); // grab mogo
    pros::delay(10);
    intakein(500); // score preload
    pros::delay(10);
    chassis.turnToPoint(-16, -24, 1000); // turn towards ring stack
    pros::delay(10);
    doinker.set_value(true); // doinker lowers
    pros::delay(1000);
    break;

    case 2: // blue mogo rush side hopefully two ring

    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -30, 1500, {.forwards = false}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(false); // grab mogo
    pros::delay(10);
    intakein(500); // score preload
    pros::delay(10);
    chassis.moveToPoint(5, -53, 1500);
    pros::delay(1000);
    break;

    case 3: // blue goal rush side alliance wall stake three ring hopefully

    moveladybrown(450, -12000);
    pros::delay(5);
    moveladybrown(450, 12000);
    pros::delay(5);
    moveladybrown(5, 0);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -6, 1000, {.forwards = false}); // back into goal
    chassis.moveToPoint(-10.5, -32, 2500, {.forwards = false, .maxSpeed = 70}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(1000);
    solenoidExtend.set_value(false); // grab mogo 
    pros::delay(5);
    chassis.moveToPoint(16, -40, 1500, {.forwards = true}); // ring stack
    pros::delay(10);
    intakein(2000); // intake blue bottom ring
    pros::delay(10);
    chassis.turnToPoint(45, -13, 300); // turn to corner
    chassis.moveToPoint(47, -12, 3000); // move to corner
    pros::delay(10);
    intakein(2500);
    backwards(200);
    forwards(250);
    intakein(1000);
    pros::delay(10);
    intakein(200);
    backwards(200);
    pros::delay(10);
    forwards(250);
    intakein(1500);
    pros::delay(10);
    intakein(200);
    backwards(200);
    pros::delay(10);
    forwards(300);
    intakein(1500);
    pros::delay(10);
    chassis.moveToPoint(-26, -14.5, 3000, {.forwards = true}); // move to ladder
    break;

    case 5: // consistent three ring blue goal rush positive
    moveladybrown(1000, -12000);
    pros::delay(10);
    moveladybrown(500, 12000);
    pros::delay(10);
    moveladybrown(10, 0);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -6, 1000, {.forwards = false, .maxSpeed = 80}); // back into goal
    pros::delay(10);
    chassis.moveToPoint(-10.5, -32, 3000, {.forwards = false, .maxSpeed = 50}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(1000);
    solenoidExtend.set_value(false); // grab mogo 
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPoint(16, -40, 1500, {.forwards = true});
    pros::delay(10);
    intakein(900);
    pros::delay(10);
    backwards(100);
    pros::delay(10);
    chassis.turnToPoint(45, -13, 300);
    pros::delay(10);
    chassis.moveToPoint(45, -13, 3000);
    pros::delay(100);
    intakein(5000);
    pros::delay(10);
    chassis.turnToPoint(-24, -14, 1000, {.maxSpeed = 70});
    pros::delay(10);
    intakeout(200);
    chassis.moveToPoint(-24, -14.5, 3000, {.forwards = true});
    intakeout(1000);
    break;

    case 6: // blue ring side, 5 ring, ladder touch

    colorsort();
    moveladybrown(600, -12000);
    pros::delay(10);
    moveladybrown(600, 12000);
    pros::delay(10);
    moveladybrown(10, 0);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -6, 500, {.forwards = false}); // back into goal
    pros::delay(10);   
    chassis.moveToPoint(10.5, -32, 1500, {.forwards = false, .maxSpeed = 80}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    solenoidExtend.set_value(false); // grab mogo 
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(-7.8, -48, 230, 4000, {.minSpeed = 70}); // move to ring stacks
    chassis.moveToPose(-12.4, -51, 230, 3000,{.minSpeed = 70}); // second movement
    intakein(4500);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(-8, -25, 0, 1000);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(-68, -16, 270, 4000); // corner
    backwards(50);
    intakein(3000);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPoint(30, -20, 4000, {.maxSpeed = 75}); // ladder
    break;
    
    case 7: // six ??

    colorsort();
    moveladybrown(600, -12000);
    pros::delay(10);
    moveladybrown(600, 12000);
    pros::delay(10);
    moveladybrown(10, 0);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -6, 500, {.forwards = false}); // back into goal
    pros::delay(10);   
    chassis.moveToPoint(10.5, -32, 1500, {.forwards = false, .maxSpeed = 80}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    solenoidExtend.set_value(false); // grab mogo 
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(-8, -48, 230, 4000, {.minSpeed = 70}); // move to ring stacks
    chassis.moveToPose(-12.4, -51, 230, 3000,{.minSpeed = 70}); // second movement
    intakein(3000);
    pros::delay(10);
    chassis.moveToPose(-8, -25, 0, 1000);
    pros::delay(10);
    chassis.moveToPose(-68, -16, 270, 3000); // corner
    pros::delay(10);
    intake.move_voltage(0);
    backwards(1000);
    pros::delay(10);
    forwards(1000);
    pros::delay(10);
    break;

    case 10: // skill?
    intakein(500);
    pros::delay(10);
    solenoidExtend.set_value(true);
    pros::delay(10);
    chassis.moveToPoint(0, 8, 400, {.minSpeed = 80});
    chassis.moveToPose(20, 8, 270, 1000, {.forwards = false, .minSpeed = 80});
    pros::delay(1000);
    solenoidExtend.set_value(true);
    pros::delay(10);
    solenoidExtend.set_value(false);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPoint(10, 25, 2000);
    pros::delay(10);
    chassis.moveToPoint(38, 38, 2000);
    pros::delay(10);
    chassis.moveToPoint(20, 38, 500);
    chassis.moveToPoint(19, 14, 2000);
    pros::delay(10);
    chassis.moveToPoint(19, 14, 2000);
    break;

    case 11: // red side mogo rush
    colorsort();
    moveladybrown(500, -12000);
    pros::delay(10);
    moveladybrown(500, 12000);
    pros::delay(10);
    moveladybrown(10, 0);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -6, 1000, {.forwards = false, .maxSpeed = 90}); // back into goal
    pros::delay(10);
    chassis.moveToPoint(10.5, -32, 2000, {.forwards = false, .maxSpeed = 70}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(100);
    solenoidExtend.set_value(false); // grab mogo 
    pros::delay(10);
    chassis.moveToPoint(-18, -40, 1500, {.forwards = true}); // ring stack
    pros::delay(10);
    intake.move_voltage(-12000);
    pros::delay(10);
    backwards(100);
    pros::delay(10);
    chassis.moveToPoint(-45, -13, 3000, {.maxSpeed = 90});
    pros::delay(100);
    intakein(4000);
    intake.move_voltage(-12000);
    pros::delay(10);
    intake.move_voltage(-12000);
    backwards(250);
    pros::delay(500);
    intake.move_voltage(-12000);
    forwards(250);
    pros::delay(1000);
    chassis.turnToPoint(-24, -14, 1000, {.maxSpeed = 70}); // to ladder
    pros::delay(10);
    chassis.moveToPoint(24, -14.5, 3000, {.forwards = true});
    break;

    case 12:
    colorsort();
    moveladybrown(600, -12000);
    pros::delay(10);
    moveladybrown(600, 12000);
    pros::delay(10);
    moveladybrown(10, 0);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -6, 500, {.forwards = false, .minSpeed = 100}); // back into goal
    chassis.moveToPoint(-12.5, -33.5, 2000, {.forwards = false, .maxSpeed = 90}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    solenoidExtend.set_value(false); // grab mogo 
    pros::delay(10);
    intakeout(50);
    intakein(50);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(7.8, -48, 130, 3500, {.minSpeed = 70}); // move to ring stacks
    chassis.moveToPose(12.8, -51, 130, 1000,{.maxSpeed = 70}); // second movement
    intakein(1000);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(8, -29, 0, 3000);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(71, -8, 85, 4000); // corner
    intakein(2000);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPoint(-28, -18, 4000, {.maxSpeed = 120}); // ladder
    ladybrown.move_voltage(-6000);
    break;

    case 13: // ring rush part 2
    colorsort();
    moveladybrown(600, -12000);
    pros::delay(10);
    moveladybrown(600, 12000);
    pros::delay(10);
    moveladybrown(10, 0);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -6, 500, {.forwards = false}); // back into goal
    chassis.moveToPoint(-11.5, -32, 1500, {.forwards = false, .maxSpeed = 80}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(100);
    solenoidExtend.set_value(false); // grab mogo 
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(7.8, -48, 130, 4000, {.minSpeed = 70}); // move to ring stacks
<<<<<<< HEAD
    chassis.moveToPose(12.4, -50, 130, 2500,{.minSpeed = 70}); // second movement
=======
    chassis.moveToPose(12.4, -50, 130, 3000,{.minSpeed = 70}); // second movement
>>>>>>> a1d2db8bf76d6da0c2ece475118d2812e96655cf
    intakein(2500);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(4, -28, 0, 1000);
    pros::delay(10);
    intake.move_voltage(-12000);
    chassis.moveToPose(70, -12, 85, 4000); // corner
    backwards(50);
    intakein(2000);
    pros::delay(10);
    next_state();
    next_state();
    intake.move_voltage(-12000);
    chassis.moveToPoint(-28, -18, 4000, {.maxSpeed = 75}); // ladder
    prev_state();
    prev_state();
    break;


  }

//chassis.turnToHeading(90, 100000)

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
  
  pros::lcd::initialize();


  while (true) {

    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    double left_speed =
        controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) +
        controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    double right_speed =
        controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) -
        controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    left_motors.move_voltage(left_speed * (12000 / 127));
    right_motors.move_voltage(right_speed * (12000 / 127));

     // Uses the L button to control hooks on intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move_voltage(intakespeed);
      coloursensor.set_led_pwm(50);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move_voltage(12000);
      coloursensor.set_led_pwm(50); 
    } else {
      
      intake.move_voltage(0);
      coloursensor.set_led_pwm(10);
    }

    // mogo mech
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      solenoidExtend.set_value(true);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      solenoidExtend.set_value(false);
    } else {
      solenoidExtend.set_value(false); //clamp automatically closes
    }
    
    // doinker - might need to change controls for yuri
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      doinker.set_value(true);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      doinker.set_value(false);
    }

    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
    // ladybug control
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
      next_state(); // forwards
    } else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
      prev_state(); // backwards
    } 

    power_wall_stake();

    pros::delay(10);

  }
}