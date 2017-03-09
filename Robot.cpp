#include <AHRS.h>
#include <CANTalon.h>
#include <WPILib.h>

//on operator side
#define INTAKESWITCH 2
#define INTAKEREV 3
#define SHOOTSWITCH 1
#define SHOOTREV 4
#define IDEALUP 12
#define IDEALDOWN 11

class Robot: public frc::IterativeRobot {
private:
	//Create objects for a driver's joystick and a second operator's joystick
	Joystick driverStick;
	Joystick operatorStick;

	//CANtalons
	CANTalon lf, lb, rf, rb, shooterMotor, intakeMotor, climberMotor;
	RobotDrive mecanumDrive;

	//IMU Sensor
	AHRS ahrs;

	//pdp used to get voltages
	PowerDistributionPanel pdp;

	double operatorThrottle;
	double driverThrottle;

	bool shootSwitch;
	bool shootSwitchPressed;
	bool shootRev;
	double shooterSpeed = 0.65;
	double kP = 0.2;
	double idealV = 13.6;

	bool intakeSwitch;
	bool intakeSwitchPressed;
	bool intakeRev;
	double intakeSpeed;

	bool driveSwitch;
	int intakeSign = 1;
	int shootSign = 1;

	double autoTime = 2;

	void updateDashboard() {
		//DisabledPeriodic may only work with LiveWindow disabled
		LiveWindow::GetInstance()->SetEnabled(false);
		//set ideal shooter speed at beginning of match
		shooterSpeed = SmartDashboard::GetNumber("Shooter Speed 0-1", 0.68) * shootSign;
		SmartDashboard::PutNumber("Shooter Speed 0-1", shooterSpeed/shootSign);
		//set proportional constant at beginning of match
		kP = SmartDashboard::GetNumber("Shooter Speed 0-1", 0.68);
		SmartDashboard::PutNumber("Shooter Speed 0-1", kP);
		//set voltage of an ideal battery at beginning of match
		idealV = SmartDashboard::GetNumber("Ideal battery voltage", 13.6);
		SmartDashboard::PutNumber("Ideal battery voltage", idealV);

		//auto end time
		autoTime = SmartDashboard::GetNumber("Auto Time", 2);
		SmartDashboard::PutNumber("Auto Time", autoTime);
	}

	void enableMotorSafety() {
		//motor safety for the drive system
		mecanumDrive.SetSafetyEnabled(true);
		mecanumDrive.SetExpiration(0.1);

		//motor safety for the shooter
		shooterMotor.SetSafetyEnabled(true);
		shooterMotor.SetExpiration(0.1);

		//motor safety for the Intake
		intakeMotor.SetSafetyEnabled(true);
		intakeMotor.SetExpiration(0.1);

		//motor safety for the Climber
		climberMotor.SetSafetyEnabled(true);
		climberMotor.SetExpiration(0.1);
	}

	void disableMotorSafety() {
		mecanumDrive.SetSafetyEnabled(false);
		shooterMotor.SetSafetyEnabled(false);
		intakeMotor.SetSafetyEnabled(false);
		climberMotor.SetSafetyEnabled(false);
	}


public:
	Robot():
		//control sticks
		driverStick(0),
		operatorStick(1),

		//drivetrain
		lf(7), // left front
		lb(4), // left back
		rf(6), // right front
		rb(3), // right back
		mecanumDrive(lf, lb, rf, rb),

		//Game Object manipulators
		shooterMotor(5),
		intakeMotor(2),
		climberMotor(1),

		//configure the gyroscope for use as a breakout on the roborio.
		ahrs(SPI::Port::kMXP)
	{
		//invert drive motors
		lf.SetInverted(true);
		lb.SetInverted(true);
	}

	void DisabledInit() {
		disableMotorSafety();
	}
	void DisabledPeriodic() {
		updateDashboard();
	}

	void AutonomousInit() {
		enableMotorSafety();
	}

	void AutonomousPeriodic() {
		updateDashboard();
		static Timer t;
		if (!t.Get()) t.Start();
		if(t.Get() < autoTime)
		{
		    mecanumDrive.MecanumDrive_Cartesian(0, -0.5, 0); //drive forwards
		}
		else {
		    mecanumDrive.MecanumDrive_Cartesian(0, 0, 0);
		}
	}

	void TeleopInit() {
		enableMotorSafety();
	}

	void TeleopPeriodic() {
		updateDashboard();
		/* Get input from Driver joystick. Changes range from -1->0, which makes sense physically
		 * Generally, the input on these joysticks is flipped such that we need to reverse it.
		 */
		driverThrottle = ((1 - driverStick.GetThrottle()) / 2);
		operatorThrottle = ((1 - operatorStick.GetThrottle()) / 2);





		/*
		 * CLIMBER CODE
		 * The climber is driven through direct control from Y-axis of the operator's joystick.
		 */
		// climber control
		climberMotor.Set(-operatorStick.GetY());
		/*
		 * END CLIMBER CODE
		 */

		/*
		 * INTAKE CODE
		 * the intake is controlled through either a toggling switch button or through a throttle.
		 * Another button is also bound to reverse the sign, or direction, of the intake motor.
		 */
		/* intake control
		 * toggled intake control with a reverse control
		 */
		if(operatorStick.GetRawButton(INTAKESWITCH) && !intakeSwitchPressed) { //toggle
			intakeSwitch = !intakeSwitch;
			intakeSwitchPressed = true;
		}
		else {
			intakeSwitchPressed = false;
		}

		intakeSign = operatorStick.GetRawButton(INTAKEREV) ? -1 : 1; // hold

		if(intakeSwitch) {
			intakeMotor.Set(intakeSpeed * intakeSign);
		} else {
			intakeMotor.Set(0);
		}
		/*
		 * END INTAKE CODE
		 */


		/*
		 * SHOOTER CODE
		 * The shooter is controlled with a toggling switch button. The speed and any proportional constants
		 * are changed through the smart dashboard.
		 *
		 * the sign, or direction, of the shooters movement can be adjusted by pressing another toggle button.
		 */

		// get shooter button
		if(operatorStick.GetRawButton(SHOOTSWITCH) && !shootSwitchPressed) { //toggle
			shootSwitch = !shootSwitch;
			shootSwitchPressed = true;
		}

		else {
			shootSwitchPressed = false;
		}

		//get shooter rev button
		shootSign = operatorStick.GetRawButton(SHOOTREV) ?  1 : -1; // hold
		//shooter control
		if(shootSwitch) {
			//pdp voltage adjusts for Voltage drops
			shooterMotor.Set(shooterSpeed * (kP * (idealV/pdp.GetVoltage())));
		} else {
			shooterMotor.Set(0);
		}
		/*
		 * END SHOOTER CODE
		 */


		/*
		 * DRIVING CODE
		 * 'car' like mecanum driving with a deadzone
		 */
		double driveX = driverStick.GetX();
		double driveY = driverStick.GetY();
		double driveZ = driverStick.GetZ();
		double deadZone = 0.1;
		double magnitude = sqrt(driveX * driveX + driveY * driveY);

		driveX /= magnitude;
		driveY /= magnitude;

		if(magnitude < deadZone) {
			magnitude = 0; //no movement in deadzone radius
		} else {
			magnitude -= deadZone; //no discontinuity
		}

		driveX *= magnitude; //scale each amount
		driveY *= magnitude;

		mecanumDrive.MecanumDrive_Cartesian(
				driverThrottle * driveX,
				driverThrottle * driveY,
				driverThrottle * driveZ);
		/*
		 * END DRIVING CODE
		 */
	}

	void TestInit() {
		disableMotorSafety(); //easier like this
	}

	void TestPeriodic() {
		updateDashboard();
		const float drivePower = 0.2;
		const float intakePower = 0.2;
		const float climbPower = 0.2;
		const float shootPower = 0.2;

		enum TEST_STATES {
			START,
			DRIVE_FORWARD,
			DRIVE_BACKWARD,
			DRIVE_LEFT,
			DRIVE_RIGHT,
			TURN_RIGHT,
			TURN_LEFT,
			INTAKE_FORWARD,
			INTAKE_BACKWARD,
			CLIMBER_FORWARD,
			CLIMBER_BACKWARD,
			SHOOTER_FORWARD,
			SHOOTER_BACKWARD,
			STOP,
			PAUSE // go to this state between changes
		};

		// switches to PAUSE between states
		static enum TEST_STATES currentState = PAUSE;
        // just iterates through the enum
		static enum TEST_STATES realState = START;
		// for edge triggering the button
		static bool switchStateToggle = false;

		if(driverStick.GetTrigger() && !switchStateToggle) {
			if (currentState == PAUSE) {
				// hacky way to increment through enum
				realState = TEST_STATES(int(realState) + 1);
				currentState = realState;
			}
			else if (currentState != STOP) {
				currentState = PAUSE;
			}
			switchStateToggle = true;
		} else if (!driverStick.GetTrigger()) {
			switchStateToggle = false;
		}

		switch (currentState) {
		DRIVE_FORWARD:
			SmartDashboard::PutString("Test Status", "Drive Forward");
			mecanumDrive.MecanumDrive_Cartesian(0, drivePower, 0);
			break;
		DRIVE_BACKWARD:
			SmartDashboard::PutString("Test Status", "Drive Backward");
			mecanumDrive.MecanumDrive_Cartesian(0, -drivePower, 0);
			break;
		DRIVE_LEFT:
			SmartDashboard::PutString("Test Status", "Drive Left");
			mecanumDrive.MecanumDrive_Cartesian(drivePower, 0, 0);
			break;
		DRIVE_RIGHT:
			SmartDashboard::PutString("Test Status", "Drive Right");
			mecanumDrive.MecanumDrive_Cartesian(-drivePower, 0, 0);
			break;
		TURN_RIGHT:
			SmartDashboard::PutString("Test Status", "Turn Right");
			mecanumDrive.MecanumDrive_Cartesian(0, 0, drivePower);
			break;
		TURN_LEFT:
			SmartDashboard::PutString("Test Status", "Turn Left");
			mecanumDrive.MecanumDrive_Cartesian(0, 0, -drivePower);
			break;
		INTAKE_FORWARD:
			SmartDashboard::PutString("Test Status", "Intake Forward");
			intakeMotor.Set(intakePower);
			break;
		INTAKE_BACKWARD:
			SmartDashboard::PutString("Test Status", "Intake Backward");
			intakeMotor.Set(-intakePower);
			break;
		CLIMBER_FORWARD:
			SmartDashboard::PutString("Test Status", "Climber Forward");
			climberMotor.Set(climbPower);
			break;
		CLIMBER_BACKWARD:
			SmartDashboard::PutString("Test Status", "Climber Backward");
			climberMotor.Set(-climbPower);
			break;
		SHOOTER_FORWARD:
			SmartDashboard::PutString("Test Status", "Shooter Forward");
			shooterMotor.Set(shootPower);
			break;
		SHOOTER_BACKWARD:
			SmartDashboard::PutString("Test Status", "Shooter Backward");
			shooterMotor.Set(-shootPower);
			break;
		START:
			SmartDashboard::PutString("Test Status", "Start");
		STOP:
			SmartDashboard::PutString("Test Status", "Stop");
		PAUSE: // stop all motors
			SmartDashboard::PutString("Test Status", "Pause");
			mecanumDrive.MecanumDrive_Cartesian(0, 0, 0);
			intakeMotor.Set(0);
			intakeMotor.Set(0);
			climberMotor.Set(0);
			climberMotor.Set(0);
			shooterMotor.Set(0);
			shooterMotor.Set(0);
			break;
		}
	}
};

START_ROBOT_CLASS(Robot)
