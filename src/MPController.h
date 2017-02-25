/*
 * MPController.h
 *
 *  Created on: Feb 18, 2017
 *      Author: Jakob
 */

#ifndef SRC_MPCONTROLLER_H_
#define SRC_MPCONTROLLER_H_

#include <WPILib.h>
#include <CANTalon.h>
#include <Profiler.h>

struct PIDF {
	double p, i, d, f;
};

class MPController {
public:
	MPController(
			std::vector<std::vector<double>> left,
			std::vector<std::vector<double>> right,
			CANTalon &leftTalon,
			CANTalon &rightTalon,
			PIDF leftPIDF,
			PIDF rightPIDF,
			double positionDeadband);

	void Control();

	void Enable();
	void Disable();

	bool OnTarget();

	void SetPIDF(PIDF pidfLeft, PIDF pidfRight) { m_pidfLeft = pidfLeft; m_pidfRight = pidfRight; }
	void SetPIDFLeft(PIDF pidfLeft) { SetPIDF(pidfLeft, m_pidfRight); }
	void SetPIDFRight(PIDF pidfRight) { SetPIDF(m_pidfLeft, pidfRight); }

private:
	Profiler profilerLeft;
	Profiler profilerRight;

	CANTalon *m_left;
	CANTalon *m_right;

	PIDF m_pidfLeft;
	PIDF m_pidfRight;

	bool isEnabled;
	double deadband;

	double leftTarget, rightTarget;

	void GetPIDF();
};



#endif /* SRC_MPCONTROLLER_H_ */
