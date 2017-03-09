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

class Robot: public frc::IterativeRobot {
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
	double shooterSpeed = 0.78;
	double kP = 0.2;
	double idealV = 13.6;

	bool shootSwitch;
	bool intakeRev;
	bool shootRev;
	bool driveSwitch;
	int intakeSign = 1;
	int shootSign = 1;

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

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() {
	}
	void TeleopInit() {
		//set ideal shooter speed at beginning of match
		double shooterSpeed = SmartDashboard::GetNumber("Shooter Speed 0-1", 0.68) * shootSign;
		//set proportional constant at beginning of match
		double kP = SmartDashboard::GetNumber("Shooter Speed 0-1", 0.68);
		//set voltage of an ideal battery at beginning of match
		double idealV = SmartDashboard::GetNumber("Ideal battery voltage", 13.6);
	}
	void TeleopPeriodic() {

		/* Get input from Driver joystick. Changes range from -1->0, which makes sense physically
		 * Generally, the input on these joysticks is flipped such that we need to reverse it.
		 */
		driverThrottle = ((1 - driverStick.GetThrottle()) / 2);
		operatorThrottle = ((1 - operatorStick.GetThrottle()) / 2);


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
		// intake control
		if(operatorStick.GetRawButton(INTAKESWITCH) && operatorThrottle == 0) {
			intakeMotor.Set(0.5 * intakeSign);
		} else {
			intakeMotor.Set(operatorThrottle * intakeSign);
		}
		/*
		 * END INTAKE CODE
		 */


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
		 * SHOOTER CODE
		 * The shooter is controlled with a toggling switch button. The speed and any proportional constants
		 * are changed through the smart dashboard.
		 *
		 * the sign, or direction, of the shooters movement can be adjusted by pressing another toggle button.
		 */
		//set ideal shooter speed
		double shooterSpeed = SmartDashboard::GetNumber("Shooter Speed 0-1", 0.68) * shootSign;
		//set proportional constant
		double kP = SmartDashboard::GetNumber("shooter proportinal const", 0.3);
		//set voltage of an ideal battery
		double idealV = SmartDashboard::GetNumber("Ideal battery voltage", 13.6);

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
		 * the driving code allows for switching between a field oriented driving style and a car-like one.
		 * switching between the modes is done through a toggling switch button.
		 */
		//reset yaw for field oriented driving
		if (driverStick.GetRawButton(YAWRESET)) {
			ahrs.ZeroYaw();
		}

		//switch between field oriented and 'car' style mecanum driving
		if(driverStick.GetRawButton(DRIVESWITCH)) {
			driveSwitch = !driveSwitch;
		}
		if (driveSwitch) {
			mecanumDrive.MecanumDrive_Cartesian(
				driverThrottle * -driverStick.GetX(),
				driverThrottle * -driverStick.GetY(),
				driverThrottle * driverStick.GetZ());
		} else if (!driveSwitch) {
			mecanumDrive.MecanumDrive_Cartesian(
				driverThrottle * -driverStick.GetX(),
				driverThrottle * -driverStick.GetY(),
				driverThrottle * driverStick.GetZ(),
				ahrs.GetAngle());
		}
		/*
		 * END DRIVING CODE
		 */
	}
private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
