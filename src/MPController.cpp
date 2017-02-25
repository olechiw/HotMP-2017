/*
 * MPController.cpp
 *
 *  Created on: Feb 18, 2017
 *      Author: Jakob
 */

#include <MPController.h>

MPController::MPController(
		std::vector<std::vector<double>> left,
					std::vector<std::vector<double>> right,
					CANTalon &leftTalon,
					CANTalon &rightTalon,
					PIDF pidfLeft,
					PIDF pidfRight,
					double positionDeadband)
:
profilerLeft(leftTalon, left),
profilerRight(rightTalon, right),
m_right(&rightTalon),
m_left(&leftTalon),
m_pidfLeft(pidfLeft),
m_pidfRight(pidfRight),
isEnabled(false),
deadband(positionDeadband)
{
	leftTarget = left[left.size()-1][0];
	rightTarget = right[right.size()-1][0];
}

bool MPController::OnTarget()
{
	/*
	bool right = false, left = false;
	double position = m_right->GetPosition();
	if ((
			(rightTarget - deadband) < position)
			&&
			(position < (rightTarget + deadband)))
		right = true;

	position = m_left->GetPosition();
	if ((
			(leftTarget - deadband) < position)
			&&
			(position < (leftTarget + deadband)))
		left = true;

	return (right && left);
	*/
	return (profilerLeft.isOver() && profilerRight.isOver());
}


void MPController::Enable()
{
	if (isEnabled)
		return;
	m_right->SetControlMode(CANTalon::kMotionProfile);
	m_left->SetControlMode(CANTalon::kMotionProfile);
	m_right->SelectProfileSlot(1);
	m_left->SelectProfileSlot(1);
	m_right->SetPID(m_pidfRight.p, m_pidfRight.i, m_pidfRight.d);
	m_left->SetPID(m_pidfLeft.p, m_pidfLeft.i, m_pidfLeft.d);
	m_right->SetF(m_pidfRight.f);
	m_left->SetF(m_pidfLeft.f);
	m_right->SetAllowableClosedLoopErr(25);
	m_left->SetAllowableClosedLoopErr(25);

	m_right->Set(profilerRight.getSetValue());
	m_left->Set(profilerLeft.getSetValue());

	//profilerLeft.start();
	profilerRight.start();
	isEnabled = true;
}

void MPController::GetPIDF()
{
	/*
	PIDF pidf = {
			SmartDashboard::GetNumber("DB/Slider 0", m_pidfLeft.p),
			SmartDashboard::GetNumber("DB/Slider 1", m_pidfLeft.i),
			SmartDashboard::GetNumber("DB/Slider 2", m_pidfLeft.d),
			SmartDashboard::GetNumber("Left F", m_pidfLeft.f)
	};
	m_pidfLeft = pidf;
	pidf = {
			SmartDashboard::GetNumber("Right P", m_pidfRight.p),
			SmartDashboard::GetNumber("Right I", m_pidfRight.i),
			SmartDashboard::GetNumber("Right D", m_pidfRight.d),
			SmartDashboard::GetNumber("Right F", m_pidfRight.f)
	};
	m_pidfRight = pidf;
	*/
}

void MPController::Control()
{
	GetPIDF();
	profilerLeft.control();
	profilerRight.control();

	if (!isEnabled)
		return;

	m_right->Set(profilerRight.getSetValue());
	m_left->Set(profilerLeft.getSetValue());

	m_right->SetControlMode(CANTalon::kMotionProfile);
	m_right->SelectProfileSlot(1);
	m_right->SetPID(m_pidfRight.p, m_pidfRight.i, m_pidfRight.d);
	m_right->SetF(m_pidfRight.f);
	m_right->SetAllowableClosedLoopErr(25);

	m_left->SetControlMode(CANTalon::kMotionProfile);
	m_left->SelectProfileSlot(1);
	m_left->SetPID(m_pidfLeft.p, m_pidfLeft.i, m_pidfLeft.d);
	m_left->SetF(m_pidfLeft.f);
	m_left->SetAllowableClosedLoopErr(25);

	m_right->Set(profilerRight.getSetValue());
	m_left->Set(profilerLeft.getSetValue());
}

void MPController::Disable()
{
	profilerLeft.reset();
	profilerRight.reset();
	//m_right->SetPosition(0);
	//m_left->SetPosition(0);
	isEnabled = false;
}
