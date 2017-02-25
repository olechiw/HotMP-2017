/**
 * This C++ FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The CANTalon class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 *
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 *
 * This application is an IterativeRobot project to demonstrate a minimal implementation not requiring the command
 * framework, however these code excerpts could be moved into a command-based project.
 *
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 *
 * Logitech Gamepad mapping, use left y axis to drive Talon normally.
 * Press and hold top-left-shoulder-button5 to put Talon into motion profile control mode.
 * This will start sending Motion Profile to Talon while Talon is neutral.
 * This will signal Talon to fire MP.  When MP is done, Talon will "hold" the last setpoint position
 * and wait for another button6 press to fire again.
 * Release button5 to allow OpenVoltage control with left y axis.
 */
#include <Profiler.h>
#include "WPILib.h"
//#include "MotionProfile.h"
#include "motionprofiles.h"
#include "CANTalon.h"
#include "RobotUtils/RobotUtils.h"
#include "MPController.h"

#define SHOOTER_P 0.001
#define SHOOTER_I 0.0
#define SHOOTER_D 0.0
#define SHOOTER_FF 0.836
#define END_POINT 14.97

const double leftMaxVelocity = 450;
const double rightMaxVelocity = 480;

const double leftEncoderRevs = 360;
const double rightEncoderRevs = 360;

const double leftFF = 1023 / ((leftMaxVelocity * (leftEncoderRevs * 4)) / 600);
const double rightFF = 1023 / ((rightMaxVelocity * (rightEncoderRevs * 4)) / 600);

const double deadband = 0;

PIDF leftPIDF =
{
		0.0,
		0.00,
		0,
		leftFF
};
PIDF rightPIDF =
{
		0,
		0,
		0,
		rightFF
};

class Robot: public IterativeRobot
{
public:
	/** The Talon we want to motion profile. */
	CANTalon m_leftTalon;
	CANTalon m_rightTalon;

	CANTalon m_leftFollower;
	CANTalon m_rightFollower;

	/** some example logic on how one can manage an MP */
	Profiler _example;

	/** joystick for testing */
	Joystick _joy;

	MPController controller;

	bool mpTrig = false;
	bool mpExit = false;

	/** cache last buttons so we can detect press events.  In a command-based project you can leverage the on-press event
	 * but for this simple example, lets just do quick compares to prev-btn-states */
	bool _btnsLast[10] = {false,false,false,false,false,false,false,false,false,false};


	Robot() :
		m_leftTalon(11),
			m_leftFollower(12),
			m_rightTalon(21),
			m_rightFollower(20),
			_example(m_leftTalon, profileLeft()),
			_joy(0),
			controller(profileRight(),
					profileLeft(),
					m_leftTalon,
					m_rightTalon,
					leftPIDF,
					rightPIDF,
					deadband)
	{
		m_leftTalon.SetFeedbackDevice(CANTalon::QuadEncoder);
		m_leftTalon.ConfigEncoderCodesPerRev(361);

		m_rightTalon.SetFeedbackDevice(CANTalon::QuadEncoder);
		m_rightTalon.ConfigEncoderCodesPerRev(341);

		m_rightTalon.SetClosedLoopOutputDirection(true);
		m_leftTalon.SetSensorDirection(true); /* keep sensor and motor in phase */

		m_leftFollower.SetControlMode(CANTalon::kFollower);
		m_leftFollower.Set(11);
		m_rightFollower.SetControlMode(CANTalon::kFollower);
		m_rightFollower.Set(21);
	}
	/**  function is called periodically during operator control */

	void TeleopInit() {
		mpTrig = false;
		mpExit = false;
		m_leftTalon.SetPosition(0.0);
		m_rightTalon.SetPosition(0.0);
		controller.Disable();
		prevAccelL = 0;
		prevAccelR = 0;
		prevVelocityL = 0;
		prevVelocityR = 0;
	}

	double prevAccelL = 0;
	double prevAccelR = 0;
	double prevVelocityL = 0;
	double prevVelocityR = 0;

	void TeleopPeriodic()
	{
		/* Instrumentation */
		double speed_rpm, error_rpm, controlMode;
		double PID_P, PID_I, PID_D, PID_FF, PID_Current, PID_Position, PID_Speed, PID_Error, PID_Setpoint;

		speed_rpm = 0.0;

		/* get buttons */
		bool btns[10];
		for(unsigned int i=1;i<10;++i)
			btns[i] = _joy.GetRawButton(i);

		//_example.control();

		controller.Control();

		if (((btns[5] == true) || (mpTrig == true)) &&
			(mpExit != true)){ /* Check button 5 */

			mpTrig = true;

			controller.Enable();

			if (controller.OnTarget())
			{
				std::cout << "On Target!" << std::endl;
				controller.Disable();
				mpTrig = false;
				mpExit = true;
			}

		} else {

			m_leftTalon.SetControlMode(CANTalon::kPercentVbus);
			m_rightTalon.SetControlMode(CANTalon::kPercentVbus);
			if (-0.1 > _joy.GetY() || 0.1 < _joy.GetY())
			{
				m_leftTalon.Set(-1*_joy.GetY());
				m_rightTalon.Set(_joy.GetY());
			}
			else
			{
				m_leftTalon.Set(0);
				m_rightTalon.Set(0);
			}
			// m_CANmotor1.Set(0);
			//_example.reset();


			if ((mpExit == true) && (btns[6] == true)) {
				mpExit = false;
				controller.Disable();
				m_leftTalon.Set(0);
				m_rightTalon.Set(0);
				m_leftTalon.SetPosition(0);
				m_rightTalon.SetPosition(0);
			}
		}


		/* save buttons states for on-press detection */
		for(int i=1;i<10;++i)
			_btnsLast[i] = btns[i];

		controlMode = m_leftTalon.GetControlMode();
		PID_Setpoint = m_leftTalon.GetSetpoint();

		double leftSpeed = m_leftTalon.GetSpeed();
		double rightSpeed = m_rightTalon.GetSpeed();

		SmartDashboard::PutNumber("Left Position", m_leftTalon.GetPosition());
		SmartDashboard::PutNumber("Right Position", m_rightTalon.GetPosition());
		SmartDashboard::PutNumber("Left Speed", leftSpeed);
		SmartDashboard::PutNumber("Right Speed", rightSpeed);

		double deltaVelocityL = prevVelocityL - leftSpeed;
		double deltaVelocityR = prevVelocityR - rightSpeed;
		prevVelocityL = leftSpeed;
		prevVelocityR = rightSpeed;


		double deltaAccelL = (prevVelocityL / .002) - prevAccelL;
		prevAccelL = (prevVelocityL / .002);
		double deltaAccelR = (prevVelocityR / .002) - prevAccelR;
		prevAccelR = (prevVelocityR / .002);
		SmartDashboard::PutNumber("Left Accel", deltaVelocityL / .002);
		SmartDashboard::PutNumber("Right Accel", deltaVelocityR / .002);
		SmartDashboard::PutNumber("Left Jerk", deltaAccelL / .002);
		SmartDashboard::PutNumber("Right Jerk", deltaAccelR / .002);

		SmartDashboard::PutNumber("Right Current", m_rightTalon.GetOutputCurrent());
		SmartDashboard::PutNumber("Left Current", m_leftTalon.GetOutputCurrent());
		SmartDashboard::PutNumber("Right Follower Current", m_rightFollower.GetOutputCurrent());
		SmartDashboard::PutNumber("Left Follower Current", m_leftFollower.GetOutputCurrent());

		SmartDashboard::PutNumber("Right Error", m_rightTalon.GetClosedLoopError());
		SmartDashboard::PutNumber("Left Error", m_leftTalon.GetClosedLoopError());

	}


	void DisabledPeriodic()
	{
		/* it's generally a good idea to put motor controllers back
		 * into a known state when robot is disabled.  That way when you
		 * enable the robot doesn't just continue doing what it was doing before.
		 * BUT if that's what the application/testing requires than modify this accordingly */
		m_leftTalon.SetControlMode(CANTalon::kPercentVbus);
		m_leftTalon.Set( 0 );
		/* clear our buffer and put everything into a known state */
		_example.reset();
	}
};

START_ROBOT_CLASS(Robot)
