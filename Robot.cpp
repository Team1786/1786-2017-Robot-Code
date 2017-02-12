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
	//Create objects for the four Talons running on a CAN Bus that we need for the drivetrain
	CANTalon *lf;
	CANTalon *lb;
	CANTalon *rf;
	CANTalon *rb;

	//Create object to control those CANTalon objects and make our mecanum lives easier
	RobotDrive *MecanumDrive;
public:
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

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
		//Range from 0 to 1
		int DriveThrottle = ((DriverStick->GetThrottle()+1)/2);


		MecanumDrive->MecanumDrive_Cartesian(DriverStick->GetX(),
											 DriverStick->GetY(),
											 DriverStick->GetZ());
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
