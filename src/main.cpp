#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"

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

/* ^^ why 9+ errors? idk. ^^ */

bool doinkerToggle = false; 

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

void moveladybrown (double time, double voltage) {
  ladybrown.move(voltage);
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
lemlib::ControllerSettings lateral_controller(3.8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              16, // derivative gain (kD)
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
int wall_stake_positions[states] = {0, -30, -150}; // angle of lb
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
  pros::Task screen_task([&]() {
      while (true) {
          
          //master.print(0, 0, "angle %d", imu.get_rotation());
          master.print(0, 0, "y: %.2f, x: %.2f", chassis.getPose().y, chassis.getPose().x); // y value works, x is questionable
          master.print(1, 0, "X: %.2f", chassis.getPose().x); // i don't think this even prints to controller
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

void autonomous(){
  chassis.moveToPoint(0, 0, 10); // resets chassis position

  int autoselect = 3; // change auton 
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

    pros::delay(3000);
    moveladybrown(1000, -12000);
    pros::delay(1000);
    moveladybrown(500, 12000);
    pros::delay(1000);
    moveladybrown(10, 0);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(10);
    chassis.moveToPoint(0, -6, 3000, {.forwards = false, .maxSpeed = 50}); // back into goal
    pros::delay(1000);
    chassis.moveToPoint(-10, -33, 3000, {.forwards = false, .maxSpeed = 50}); // back into goal
    pros::delay(1000);
    solenoidExtend.set_value(true); // open clamp
    pros::delay(1000);
    solenoidExtend.set_value(false); // grab mogo 
    pros::delay(10);
    chassis.moveToPoint(12, -45, 1500, {.forwards = true});
    pros::delay(10);
    intakein(2300);
    pros::delay(10);
    chassis.moveToPoint(-24, -14, 5000, {.forwards = true});
    

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
	pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
			pros::lcd::print(4, "left: %.2f, right: %.2f", ladybrown.get_position(0), ladybrown.get_position(1));
			
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
		}
    });

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
      intake.move_voltage(-12000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move_voltage(12000);
    } else {
      intake.move_voltage(0);
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

    /* lemlib opcontrol - won't use.

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