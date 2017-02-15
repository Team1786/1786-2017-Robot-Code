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

class Robot: public frc::IterativeRobot {
	//Create objects for a driver's joystick and a second operator's joystick
	Joystick *DriverStick;
	Joystick *OperatorStick;

	//Cantalons
	CANTalon *lf, *lb, *rf, *rb, *ShooterMotor, *IntakeMotor;
	RobotDrive *MecanumDrive;

	double OperatorThrottle, OperatorX, OperatorY, OperatorZ;
	double DriverThrottle, DriverX, DriverY, DriverZ;

public:
	void RobotInit() {
		//Create instances of the previously defined joysticks with their ID's
		DriverStick = new Joystick(0);
		OperatorStick = new Joystick(1);

		//Create instances of the four CANTalons we defined, with their ID's as constructor
		//params
		// lf = left front, lb = left back, and so on
		lf = new CANTalon(3);
		lb = new CANTalon(4);
		rf = new CANTalon(2);
		rb = new CANTalon(5);

		ShooterMotor = new CANTalon(6);
		IntakeMotor = new CANTalon(7);
		ShooterMotor->SetSafetyEnabled(true);
		ShooterMotor->SetExpiration(0.1);
		IntakeMotor->SetSafetyEnabled(true);
		IntakeMotor->SetExpiration(0.1);
		//Create instance of Robotdrive we defined earlier, uses the 4 Cantalons as params
		MecanumDrive = new RobotDrive(lf, lb, rf, rb);
		MecanumDrive->SetSafetyEnabled(true);
		MecanumDrive->SetExpiration(0.1);
		lf->SetInverted(true);
		lb->SetInverted(true);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

		//Get input from Operator joystick and put them onto the dashboard
		OperatorX = OperatorStick->GetX();
		OperatorY = OperatorStick->GetY();
		OperatorZ = OperatorStick->GetZ();
		OperatorThrottle = OperatorStick->GetThrottle();
		SmartDashboard::PutNumber("Operator X", OperatorX);
		SmartDashboard::PutNumber("Operator Y", OperatorY);
		SmartDashboard::PutNumber("Operator Z", OperatorZ);
		SmartDashboard::PutNumber("Operator Throttle", OperatorThrottle);

		//Get input from Driver joystick and put them onto the dashboard
		DriverX = DriverStick->GetX();
		DriverY = DriverStick->GetY();
		DriverZ = DriverStick->GetZ();
		DriverThrottle = DriverStick->GetThrottle();
		SmartDashboard::PutNumber("Driver X", DriverX);
		SmartDashboard::PutNumber("Driver Y", DriverY);
		SmartDashboard::PutNumber("Driver Z", DriverZ);
		SmartDashboard::PutNumber("Driver Throttle", DriverThrottle);

		//Range from 0 to 1
		MecanumDrive->MecanumDrive_Cartesian(DriverX,
											 DriverY,
											 DriverZ);

		//quick and dirty shooter testing
		ShooterMotor->Set(OperatorThrottle);

		//quick and dirty shooter testing
		ShooterMotor->Set(DriverThrottle);
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
