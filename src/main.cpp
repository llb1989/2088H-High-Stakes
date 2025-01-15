#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

pros::MotorGroup left_motors({3, -4, -17}, pros::MotorGearset::blue); 
pros::MotorGroup right_motors({-11, 19, 20}, pros::MotorGearset::blue); 

pros::Motor intakehooks(10); // all in one motor
pros::Motor intakewheels(9);


/* Pneumatics! */
//#define DIGITAL_SENSOR_PORT 'A'
pros::adi::DigitalOut solenoidExtend('A');

pros::Imu imu(1);

/* pros::Rotation vertical_encoder(20);
 //vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_325, -6); */


void forwards(double time) {	// move forwards
	left_motors.move_voltage(12000);
	right_motors.move_voltage(12000);
	pros::delay(time);
	left_motors.move_voltage(0);
	right_motors.move_voltage(0);
}

void backwards(double time) {	// move backwards
	left_motors.move_voltage(-12000);
	right_motors.move_voltage(-12000);
	pros::delay(time);
	left_motors.move_voltage(0);
	right_motors.move_voltage(0);
}

void left (double time) {	// turn left
	left_motors.move_voltage(-12000);
	right_motors.move_voltage(12000);
	pros::delay(time);
	left_motors.move_voltage(0);
	right_motors.move_voltage(0);
}

void right (double time) {	// turn right
	left_motors.move_voltage(12000);
	right_motors.move_voltage(-12000);
	pros::delay(time);
	left_motors.move_voltage(0);
	right_motors.move_voltage(0);
}

void openmogo (double time) { // opens mogo mech
	solenoidExtend.set_value(true);
	pros::delay(time);
	solenoidExtend.set_value(false);
}

void closemogo (double time) {	// clses mogo mech
	solenoidExtend.set_value(true);
	pros::delay(time);
	solenoidExtend.set_value(false);
}

void intakeauton(double time) { // both hooks and intake wheels
	intakewheels.move_voltage(9000);
	intakehooks.move_voltage(9000);
	pros::delay(time);
	intakewheels.move_voltage(0);
	intakehooks.move_voltage(0);
}

lemlib::Drivetrain drivetrain(&left_motors, 
                              &right_motors, 
                              13.5, // change track width once built
                              lemlib::Omniwheel::NEW_325,
                              360,
                              2 // horizontal drift is 2 (for now)
);

// odometry settings
lemlib::OdomSensors sensors(nullptr, 
                            nullptr, 
                            nullptr, 
                            nullptr, 
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(0.6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(0.8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0.2, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
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
    chassis.calibrate(); // calibrate sensors
	pros::delay(2000);

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


void autonomous() {
	
	int whatauton = 6; 
    switch (whatauton) { // auton selecor switch statement
		case 5: //lem the lib
		 // set position to x:0, y:0, heading:0s
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.moveToPoint(0, 48, 10000);


		break;
		
		case 6:
solenoidExtend.set_value(true); 
	backwards(550);
	pros::delay(10);
	solenoidExtend.set_value(false); // we repeat this twice to be safe
	pros::delay(10);
	solenoidExtend.set_value(false);
	forwards(200);
	pros::delay(10);
	intakeauton(1000); // preload onto mobile goal
	pros::delay(10);
	left(150);
	pros::delay(10);
	solenoidExtend.set_value(false);
	forwards(250);
	pros::delay(10);
	intakeauton(1000); 
	pros::delay(10);
	intakeauton(1000);
	pros::delay(10);
	left(200);

		break;
		case 1: //one ball timed both
	
	solenoidExtend.set_value(true); 
	backwards(550);
	pros::delay(20);
	solenoidExtend.set_value(false); // we repeat this twice to be safe
	pros::delay(20);
	solenoidExtend.set_value(false);
	forwards(200);
	pros::delay(20);
	intakeauton(2000); // preload onto mobile goal
		break;

		case 2: // timed skills
	solenoidExtend.set_value(true); // process of getting one-ring
	backwards(200);
	pros::delay(20);
	solenoidExtend.set_value(false); 
	pros::delay(20);
	solenoidExtend.set_value(false);
	forwards(200);
	pros::delay(20);
	intakeauton(2000);
	pros::delay(20); 
	backwards(250); // starts moving towards second ring
	pros::delay(20);
	intakeauton(200);
	left(250);
	intakeauton(200);
	pros::delay(20);
	forwards(700);
	intakeauton(700); 
	pros::delay(20);
	intakeauton(2000);
	pros::delay(20);
	forwards(100);			
	right(100);
	intakeauton(2000);
	pros::delay(20);
	intakeauton(200);
	left(570);
	pros::delay(20);
	backwards(1000);
	pros::delay(200);	
	openmogo(500);	// drops mogo onto grey foam tile
	forwards(100);
	backwards(100);
	openmogo(500);
	pros::delay(1000);
		break;

		case 3: //two ring? timed right
	solenoidExtend.set_value(true);
	backwards(550);
	pros::delay(20);
	solenoidExtend.set_value(false);
	pros::delay(20);
	solenoidExtend.set_value(false);
	forwards(200);
	pros::delay(20);
	intakeauton(2000);
	pros::delay(20);
	left(350);	// robot starts turning towards another ring here
	forwards(500);
	intakeauton(2000);	
		break;

		case 4: //two ring? timed left
	
	solenoidExtend.set_value(true);
	backwards(550);
	pros::delay(20);
	solenoidExtend.set_value(false);
	pros::delay(20);
	solenoidExtend.set_value(false);
	forwards(200);
	pros::delay(20);
	intakeauton(2000);
	pros::delay(20);
	right(350);	// robot starts turning towards another ring here
	forwards(500);
	intakeauton(2000);	
		break; 
	} 
    
}



/* void autonomous(){
	
	chassis.calibrate();
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
	pros::delay(20);
	chassis.turnToHeading(200, 1000);

	//chassis.turnToHeading(90, 2000);
	pros::delay(20);

	int auton = 1; 
    switch (auton) {
		case 1:
		break;
	} 

 }
*/
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

	while (true){
		#define DIGITAL_SENSOR_PORT 'A'
		pros::Controller controller (pros::E_CONTROLLER_MASTER); 

		double left_speed = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)+controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		double right_speed = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)-controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		left_motors.move_voltage(left_speed*(12000/127));
		right_motors.move_voltage(right_speed*(12000/127));

		// Uses the R buttons to control the wheels on intake

		// Uses the L button to control hooks on intake
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intakehooks.move_voltage(8000); // in hooks
			intakewheels.move_voltage(8000);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intakehooks.move_voltage(-8000);
			intakewheels.move_voltage(-8000);
		} else{
			intakehooks.move_voltage(0);
			intakewheels.move_voltage(0);
		}

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			solenoidExtend.set_value(true);
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			solenoidExtend.set_value(false);
		} else {
			solenoidExtend.set_value(false);
			
		}


		pros::delay(20);


	/*pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	} */
	}
}