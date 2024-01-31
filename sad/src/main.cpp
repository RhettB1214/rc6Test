#include "main.h"
#include "lemlib/api.hpp"
#include "autoSelect/selection.h"
#include "definitions.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"




/*ASSET Defintions*/

	/*Red Far*/
	ASSET(RedFar1_txt);
	ASSET(RedFar2_txt);
	ASSET(RedFar3_txt);
	ASSET(RedFar4_txt);

	/*Red Close*/
	ASSET(Close1_txt);
	ASSET(Close2_txt);
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
	bool lastKnownButtonYState;
	bool lastKnownButtonRightState;
	bool lastKnownButtonDownState;
	bool wingToggle = false;
	bool blockerToggle = false;
	bool hangToggle = false;

	/*Kicker Variables*/
	bool slapperFireToggle = false;
	bool kickerSet = false;




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

	pros::Motor slapperMotor(SLAP_PORT, TORQUEBOX, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor intakeMotor(INTAKE_PORT, SPEEDBOX, false);

	pros::Distance kickerDistance(DISTANCE_PORT);

	pros::ADIDigitalOut vertWingPnuem(VERTW_ADIDO);
	pros::ADIDigitalOut horiWingPnuem(HORIW_ADIDO);
	pros::ADIDigitalOut blockerPnuem(BLOCKER_ADIDO);
	pros::ADIDigitalOut hangPnuem(HANG_ADIDO);



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
		8,  //16, // kP
        0, //3 // kI
		32, //80, // kD
        0, //4 // Windup Range
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
    );
	/*End of Lateral (Forwards/Backwards) PID Initilization*/


	/*Angular (Turning) PID Initilization*/
	lemlib::ControllerSettings angularController(
		4,  //7 // kP
        0, // kI
		40, //60 // kD
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


void slapperControl()
{
	while (kickerDistance.get() <= 10)
	{
		slapperMotor.move(127);
	}
	if(kickerDistance.get() > 10)
	{
		slapperMotor.move(0);
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
	selector::init();
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

	switch(selector::auton)
	{
		case 1:
			drive.setPose(48.026, -55.261, 318);
			horiWingPnuem.set_value(1);
			pros::delay(300);
			horiWingPnuem.set_value(0);
			intakeMotor.move(127);
			drive.follow(RedFar1_txt, 15, 2000);
			drive.waitUntilDone();
			vertWingPnuem.set_value(1);
			pros::delay(200);
			drive.moveToPoint(50, 0, 1000, {.forwards = false});
			drive.waitUntilDone();
			drive.moveToPoint(25, 0, 500,{.forwards = true});
			drive.waitUntilDone();
			vertWingPnuem.set_value(0);
			pros::delay(600);
			drive.moveToPoint(50, 0, 1500);
			drive.moveToPoint(35, 0, 500, {.forwards = false});
			drive.turnTo(35, -60, 500, {});
			drive.follow(RedFar2_txt, 15, 2250);
			drive.waitUntilDone();
			drive.moveToPoint(25, -24, 1000, {.forwards = false});
			drive.turnTo(65, -65, 500, {});
			drive.waitUntilDone();
			pros::delay(250);
			drive.follow(RedFar3_txt, 15, 4350);
			drive.waitUntilDone();
			drive.moveToPose(poseX, poseY, 45, 500);
			drive.waitUntilDone();
			vertWingPnuem.set_value(1);
			pros::delay(200);
			drive.moveToPose(poseX, poseY, -45, 500);
			drive.waitUntilDone();
			intakeMotor.move(-127);
			vertWingPnuem.set_value(0);
			pros::delay(200);
			horiWingPnuem.set_value(1);
			pros::delay(200);
			horiWingPnuem.set_value(0);
			drive.moveToPose(poseX, poseY, 35, 500);
			drive.waitUntilDone();
			drive.moveToPoint(60, -24, 1000, {.minSpeed = 127});
			drive.waitUntilDone();
			drive.moveToPoint(60, -32, 1000, {.forwards = false, .minSpeed = 127});
			drive.moveToPoint(60, -24, 1000, {.minSpeed = 127});
			break;

		case 2:
			drive.setPose({-53, -52, -45});
			drive.follow(Close1_txt, 15, 2000, false);
			drive.follow(Close2_txt, 15, 2000);
			vertWingPnuem.set_value(1);
			pros::delay(200);
			drive.turnTo(65, -64, 500, {});
			drive.waitUntilDone();
			vertWingPnuem.set_value(0);
			drive.moveToPoint(-11, -60, 2000);
			/*drive.follow(RedClose1_txt, 15, 2000);
			drive.waitUntil(26);
			intakeMotor.move(-127);
			drive.waitUntilDone();
			intakeMotor.move(0);
			pros::delay(1000);
			drive.follow(RedClose2_txt, 15, 1500, false);
			drive.waitUntil(4);
			vertWingPnuem.set_value(1);
			drive.waitUntilDone();
			vertWingPnuem.set_value(0);
			pros::delay(500);
			drive.follow(RedClose3_txt, 15, 2000);
			drive.waitUntilDone();
			drive.follow(RedClose4_txt, 15, 1500, false);
			drive.waitUntilDone();*/
			break;
	}
	
	
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
	lDrive.set_brake_modes(COAST);
	rDrive.set_brake_modes(COAST);

	pros::Task kickerTask(slapperControl);
	//load();

	while(true)
	{
		
		std::cout << "Distance: " << kickerDistance.get() << " Motor Position: " << slapperMotor.get_position() << std::endl;

		drive.tank(masterLeftY, masterRightY, 5);

		


		if (masterL1)
		{
			intakeMotor.move(-127);
		}
		else if (masterL2) 
		{
			intakeMotor.move(127);
		}
		else
		{
			intakeMotor.move(0);
		}


		if (masterR1 != lastKnownButtonR1State)
		{
			lastKnownButtonR1State = masterR1;
			if (masterR1)
			{
				slapperFireToggle = !slapperFireToggle;
			}
		}

		if (slapperFireToggle)
		{
			slapperMotor.move(115);
		}
		else
		{
			slapperMotor.move(0);
		}

		if (masterR2)
		{
			horiWingPnuem.set_value(1);
		}
		else
		{
			horiWingPnuem.set_value(0);
		}

		// Vertical Wing Pneumatics Toggle
		if(masterY != lastKnownButtonYState)
		{
			lastKnownButtonYState = masterY;
			if(masterY)
			{
				wingToggle = !wingToggle;
				vertWingPnuem.set_value(wingToggle);
			}
		}


		if(masterRight != lastKnownButtonRightState)
		{
			lastKnownButtonRightState = masterRight;
			if(masterRight)
			{
				blockerToggle = !blockerToggle;
				blockerPnuem.set_value(blockerToggle);
			}
		}

		if(masterDown != lastKnownButtonDownState)
		{
			lastKnownButtonDownState = masterDown;
			if(masterDown && blockerToggle == false)
			{
				blockerToggle = !blockerToggle;
				blockerPnuem.set_value(blockerToggle);
			}
			else if(masterDown && blockerToggle == true)
			{
				hangToggle = !hangToggle;
				hangPnuem.set_value(hangToggle);
				blockerToggle = !blockerToggle;
				blockerPnuem.set_value(blockerToggle);
				
			}
			
		}



		





	}
	
}
