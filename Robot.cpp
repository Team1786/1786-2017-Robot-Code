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

	//pdp used to get voltages
	PowerDistributionPanel *pdp;

	double OperatorThrottle;
	double DriverThrottle;
	double shooterSpeed = 0.78;
	double kP = 0.2;
	double idealV = 13.6;

	bool intakeSwitch;
	bool shootSwitch;
	bool intakeRev;
	bool shootRev;
	bool driveSwitch;
	int intakeSign = 1;
	int shootSign = 1;

public:
	void RobotInit() {
		DriverStick = new Joystick(0);
		OperatorStick = new Joystick(1);

		/*
		 * lf = left front
		 * lb = left back
		 * rf = right front
		 * rb = right back
		 */
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

		//motor safety for the drive system, and assign CANTalons
		MecanumDrive = new RobotDrive(lf, lb, rf, rb);
		MecanumDrive->SetSafetyEnabled(true);
		MecanumDrive->SetExpiration(0.1);

		//configure the gyroscope for use as a breakout on the roborio.
		ahrs = new AHRS(SPI::Port::kMXP);
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
		DriverThrottle = ((DriverStick->GetThrottle()-1)/-2);
		OperatorThrottle = ((OperatorStick->GetThrottle()-1)/-2);


		/*
		 * INTAKE CODE
		 * the intake is controlled through either a toggling switch button or through a throttle.
		 * Another button is also bound to reverse the sign, or direction, of the intake motor.
		 */
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
		/*
		 * END INTAKE CODE
		 */


		/*
		 * CLIMBER CODE
		 * The climber is driven through direct control from Y-axis of the operator's joystick.
		 */
		// climber control
		ClimberMotor->Set(-OperatorStick->GetY());
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
		if(OperatorStick->GetRawButton(SHOOTSWITCH) && !shootSwitch) {
			shootSwitch = true;
		} else if(OperatorStick->GetRawButton(SHOOTSWITCH) && shootSwitch) {
			shootSwitch = false;
		}

		//get shooter rev button
		if(OperatorStick->GetRawButton(SHOOTREV) && !shootRev) {
			shootSign *= -1;
			shootRev = true;
		} else if (!OperatorStick->GetRawButton(SHOOTREV)) {
			shootRev = false;
		}
		//shooter control
		if(shootSwitch) {
			//pdp voltage adjusts for Voltage drops
			ShooterMotor->Set(shooterSpeed * (kP * (idealV/pdp->GetVoltage()));
		} else {
			ShooterMotor->Set(0);
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
