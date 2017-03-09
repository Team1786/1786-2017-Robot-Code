#include <iostream>
#include <memory>
#include <string>
#include <CANTalon.h>
#include <WPILib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <cmath>
#include <AHRS.h>

//on driver side
#define YAWRESET 12
#define DRIVESWITCH 11

//on operator side
#define INTAKESWITCH 12
#define INTAKEREV 11
#define SHOOTSWITCH 7
#define SHOOTREV 6
#define CLIMBUP 8
#define IDEALUP 9
#define IDEALDOWN 10

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
	PowerDistributionPanel *pdp;

	double operatorThrottle;
	double driverThrottle;

	bool shootSwitch;
	bool shootRev;
	double shooterSpeed = 0.78;
	double kP = 0.2;
	double idealV = 13.6;

	bool intakeSwitch;
	bool intakeRev;
	double intakeSpeed;

	bool driveSwitch;
	int intakeSign = 1;
	int shootSign = 1;

	void updateDashboard() {
		//DisabledPeriodic may only work with LiveWindow disabled
		LiveWindow::GetInstance()->SetEnabled(false);

		SmartDashboard::PutNumber("Shooter Speed 0-1", 0.68);
		SmartDashboard::PutNumber("Ideal battery voltage", 13.6);
		SmartDashboard::PutNumber("Ideal battery voltage", 13.6);
		//set ideal shooter speed at beginning of match
		double shooterSpeed = SmartDashboard::GetNumber("Shooter Speed 0-1", 0.68) * shootSign;
		//set proportional constant at beginning of match
		double kP = SmartDashboard::GetNumber("Proportional shoot constant", 0.3);
		//set voltage of an ideal battery at beginning of match
		double idealV = SmartDashboard::GetNumber("Ideal battery voltage", 13.6);
	}
public:
	Robot():
		driverStick(0),
		operatorStick(1),

		/*
		 * lf = left front
		 * lb = left back
		 * rf = right front
		 * rb = right back
		 */
		lf(5),
		lb(3),
		rf(2),
		rb(4),
		mecanumDrive(lf, lb, rf, rb),

		//Game Object manipulators
		shooterMotor(6),
		intakeMotor(7),
		climberMotor(8),

		//configure the gyroscope for use as a breakout on the roborio.
		ahrs(SPI::Port::kMXP)
	{
		//invert drive motors
		lf.SetInverted(true);
		lb.SetInverted(true);

		//motor safety for the drive system, and assign CANTalons
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
	void DisabledInit() {
		mecanumDrive.SetSafetyEnabled(false);
	}
	void DisabledPeriodic() {
		updateDashboard();
	}
	void AutonomousInit() override {
	}

	void AutonomousPeriodic() {
	}
	void TeleopInit() {

	}
	void TeleopPeriodic() {

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
		//get intake rev button
		if(operatorStick.GetRawButton(INTAKEREV) && !intakeRev) {
			intakeSign *= -1;
			intakeRev = true;
		} else if (!operatorStick.GetRawButton(INTAKEREV)) {
			intakeRev = false;
		}
		/* intake control
		 * toggled intake control with a reverse control
		 */
		if(operatorStick.GetRawButton(INTAKESWITCH)) {
			intakeSwitch = !intakeSwitch;
		}
		if(operatorStick.GetRawButton(INTAKEREV)) {
			intakeSign = -1;
		} else {
			intakeSign = 1;
		}
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
		if(operatorStick.GetRawButton(SHOOTSWITCH)) {
			shootSwitch = !shootSwitch;
		}
		//get shooter rev button
		if(operatorStick.GetRawButton(SHOOTREV) && !shootRev) {
			shootSign *= -1;
			shootRev = true;
		} else if (!operatorStick.GetRawButton(SHOOTREV)) {
			shootRev = false;
		}
		//shooter control
		if(shootSwitch) {
			//pdp voltage adjusts for Voltage drops
			shooterMotor.Set(shooterSpeed * (kP * (idealV/pdp->GetVoltage())));
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
};

START_ROBOT_CLASS(Robot)
