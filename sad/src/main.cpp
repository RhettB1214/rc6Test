#include "main.h"
#include "lemlib/api.hpp"
#include "definitions.hpp"

//ASSET Defintions

	/*Red Far*/
	ASSET(penis_txt);

	/*Red Close*/
	ASSET(RedClose1_txt);
	ASSET(RedClose2_txt);
	ASSET(RedClose3_txt);
	ASSET(RedClose4_txt);

	/*Red Safe AWP Far*/

	/*Blue Far*/


	/*Blue Close*/

	
	/*Blue Safe AWP Far*/



	/*Skills*/


/*End of ASSET Defintions*/

/*Variable Definitions*/

    /*Controller Variables*/
    bool lastKnownButtonR1State;
	bool lastKnownButtonBState;
	bool lastKnownButtonUpState;
	bool wingToggle = false;
	bool actuatateWings = false;

	/*Cata Variables*/
	bool slapperFireToggle = false;


/*End of Variable Definitions*/


/*Device Initilization*/

	/*Drivetrain Initilizations*/

	pros::Motor lD1(LD1, SPEEDBOX, true);
	pros::Motor lD2(LD2, SPEEDBOX, true);
	pros::Motor lD3(LD3, SPEEDBOX, true);
	pros::Motor rD1(RD1, SPEEDBOX, false);
	pros::Motor rD2(RD2, SPEEDBOX, false);
	pros::Motor rD3(RD3, SPEEDBOX, false);

	pros::MotorGroup lDrive({lD1, lD2, lD3});
	pros::MotorGroup rDrive({rD1, rD2, rD3});

	pros::Imu imu(IMU_PORT);

	pros::Rotation odomRot(ODOM_ROT, false);

	lemlib::TrackingWheel odomWheel(&odomRot, 2.75, 0, 1);

	/*End of Drivetrain Initializations*/


	/*Non-DT Initializations*/

	pros::Motor slapperMotor(SLAP_PORT, TORQUEBOX, true);
	pros::Motor intakeMotor(INTAKE_PORT, SPEEDBOX, false);

	pros::ADIDigitalOut wingPnuem(WING_ADIDO);



	/*End of Non-DT Initializations*/

	/*Controller Initialization*/

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	/*End of Controller Initilization*/


/*End of Device Initilization*/


/*LemLib Chassis Initializations*/

	/*LemLib Drivetrain Initilization*/
	lemlib::Drivetrain drivetrain
	{
		&lDrive, /*Pointer to the left drive channel*/
		&rDrive, /*Pointer to the right drive channel*/
		10.5, /*Track Width*/
		3.25, /*Wheel Diameter*/
		450, /*Wheel RPM*/
		8 /*Chase Power*/
	};
	/*End of LemLib Drivetrain Initilization*/


	/*LemLib Odometry Initilization*/
	lemlib::OdomSensors odomSensors
	{
		&odomWheel, /*Center Wheel*/
		nullptr, /*No Tracking Wheel*/
		nullptr, /*No Tracking Wheel*/
		nullptr, /*No Tracking Wheel*/
		&imu /*Inertial Sensor*/
	};
	/*End of LemLib Odometery Sensors Initilization*/


	/*Lateral (Forwards/Backwards) PID Initilization*/
	lemlib::ControllerSettings lateralController
	(
		16,  //16, // kP
        3, // kI
		80, //72, // kD
        4, // Windup Range
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
    );
	/*End of Lateral (Forwards/Backwards) PID Initilization*/


	/*Angular (Turning) PID Initilization*/
	lemlib::ControllerSettings angularController(
		7,  //10 // kP
        0, // kI
		60, //60 // kD
        0, // Windup Range
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
    );
	/*End of Angular (Turning) PID Initilization*/


	/*LemLib Chassis Initilization*/
	lemlib::Chassis drive(drivetrain, lateralController, angularController, odomSensors);
	/*End of LemLib Chassis Initilization*/


/*End of LemLib Chassis Initializations*/


void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = drive.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	lemlib::infoSink()->setLowestLevel(lemlib::Level::DEBUG);
	drive.calibrate();
	intakeMotor.set_brake_mode(HOLD);
	slapperMotor.set_brake_mode(COAST);	
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
void autonomous() 
{
	lDrive.set_brake_modes(HOLD);
	rDrive.set_brake_modes(HOLD);

	drive.setPose(14, -56.7, 270);
	intakeMotor.move(127);
	drive.moveToPose(8, -60, 270, 1000, {.minSpeed = 100});
	drive.waitUntilDone();
	pros::delay(150);
	drive.moveToPose(14, -60, 270, 1000);
	drive.waitUntilDone();
	drive.follow(penis_txt, 25, 10000, false);
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
void opcontrol() 
{
	
}
