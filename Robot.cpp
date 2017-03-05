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
	Joystick *DriverStick;
	Joystick *OperatorStick;

	//CANtalons
	CANTalon *lf, *lb, *rf, *rb, *ShooterMotor, *IntakeMotor, *ClimberMotor;
	RobotDrive *MecanumDrive;

	//IMU Sensor
	AHRS *ahrs;

	double OperatorThrottle;
	double DriverThrottle;

	bool intakeSwitch;
	bool shootSwitch;
	bool intakeRev;
	bool shootRev;
	bool driveSwitch;
	int intakeSign = 1;
	int shootSign = 1;
public:
	void RobotInit() {
		//Create instances of the previously defined joysticks with their ID's
		DriverStick = new Joystick(0);
		OperatorStick = new Joystick(1);

		//Create instances of the four CANTalons we defined, with their ID's as constructor
		//params
		// lf = left front, lb = left back, and so on
		lf = new CANTalon(5);
		lb = new CANTalon(3);
		rf = new CANTalon(2);
		rb = new CANTalon(4);
		lf->SetInverted(true);
		lb->SetInverted(true);

		//Game Object manipulators
		ShooterMotor = new CANTalon(6);
		IntakeMotor = new CANTalon(7);
		ClimberMotor = new CANTalon(8);

		//motor safety for the shooter
		ShooterMotor->SetSafetyEnabled(true);
		ShooterMotor->SetExpiration(0.1);

		//motor safety for the Intake
		IntakeMotor->SetSafetyEnabled(true);
		IntakeMotor->SetExpiration(0.1);

		//motor safety for the Climber
		ClimberMotor->SetSafetyEnabled(true);
		ClimberMotor->SetExpiration(0.1);

		//Create instance of Robotdrive we defined earlier, uses the 4 Cantalons as params
		MecanumDrive = new RobotDrive(lf, lb, rf, rb);
		MecanumDrive->SetSafetyEnabled(true);
		MecanumDrive->SetExpiration(0.1);

		ahrs = new AHRS(SPI::Port::kMXP);
	}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() {
	}
	void TeleopInit() {

	}
	void TeleopPeriodic() {

		//Get input from Driver joystick. Changes range from -1->0, which makes sense physically
		DriverThrottle = ((DriverStick->GetThrottle()-1)/-2);
		OperatorThrottle = ((OperatorStick->GetThrottle()-1)/-2);

		//get intake button
		if(OperatorStick->GetRawButton(INTAKESWITCH)) {
			intakeSwitch = true;
		} else if(!OperatorStick->GetRawButton(INTAKESWITCH)) {
			intakeSwitch = false;
		}
		//get intake rev button
		if(OperatorStick->GetRawButton(INTAKEREV) && !intakeRev) {
			intakeSign = intakeSign * -1;
			intakeRev = true;
		} else if (!OperatorStick->GetRawButton(INTAKEREV)) {
			intakeRev = false;
		}
		// intake control
		if(intakeSwitch && OperatorThrottle == 0) {
			IntakeMotor->Set(0.5 * intakeSign);
		} else {
			IntakeMotor->Set(OperatorThrottle * intakeSign);
		}

		// get shooter button
		if(OperatorStick->GetRawButton(SHOOTSWITCH)) {
			shootSwitch = true;
		} else if(!OperatorStick->GetRawButton(SHOOTSWITCH)) {
			shootSwitch = false;
		}
		//get shooter rev button
		if(OperatorStick->GetRawButton(SHOOTREV) && !shootRev) {
			shootSign *= -1;
			shootRev = true;
		} else if (!OperatorStick->GetRawButton(SHOOTREV)) {
			shootRev = false;
		}
		// climber control
		ClimberMotor->Set(-OperatorStick->GetY());

		//reset yaw for field oriented driving
		bool reset_yaw_button_pressed = DriverStick->GetRawButton(YAWRESET);
		if ( reset_yaw_button_pressed ) {
			ahrs->ZeroYaw();
		}

		//switch between field oriented and 'car' style mecanum driving
		if(DriverStick->GetRawButton(DRIVESWITCH) && !driveSwitch) {

			driveSwitch = true;
		} else if(DriverStick->GetRawButton(DRIVESWITCH) && driveSwitch) {
			driveSwitch = false;
		}
		if (driveSwitch) {
			MecanumDrive->MecanumDrive_Cartesian(DriverThrottle * -DriverStick->GetX(),
												 DriverThrottle * -DriverStick->GetY(),
												 DriverThrottle * DriverStick->GetZ());
		} else if (!driveSwitch) {
			MecanumDrive->MecanumDrive_Cartesian(DriverThrottle * -DriverStick->GetX(),
												 DriverThrottle * -DriverStick->GetY(),
												 DriverThrottle * DriverStick->GetZ(),
												 ahrs->GetAngle());
		}
	}
private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
