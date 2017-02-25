#ifndef MotionProfileExample__h_
#define MotionProfileExample__h_
/**
 * Example logic for firing and managing motion profiles.
 * This example sends MPs, waits for them to finish
 * Although this code uses a CANTalon, nowhere in this module do we changeMode() or call set() to change the output.
 * This is done in Robot.java to demonstrate how to change control modes on the fly.
 * 
 * The only routines we call on Talon are....
 * 
 * changeMotionControlFramePeriod
 * 
 * getMotionProfileStatus		
 * clearMotionProfileHasUnderrun     to get status and potentially clear the error flag.
 * 
 * pushMotionProfileTrajectory
 * clearMotionProfileTrajectories
 * processMotionProfileBuffer,   to push/clear, and process the trajectory points.
 * 
 * getControlMode, to check if we are in Motion Profile Control mode.
 * 
 * Example of advanced features not demonstrated here...
 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position. 
 */
#include <MotionProfile.h>
#include "WPILib.h"
#include "CANTalon.h"

class Profiler
{
public:

	CANTalon::MotionProfileStatus _status;

	CANTalon & _talon;

	int _state = 0;

	bool _bStart = false;

	bool _over = false;

	CANTalon::SetValueMotionProfile _setValue = CANTalon::SetValueMotionProfileDisable;

	static const int kMinPointsInTalon = 5;

	void PeriodicTask()
	{
		/* keep Talons happy by moving the points from top-buffer to bottom-buffer */
		_talon.ProcessMotionProfileBuffer();
	}

	std::vector<std::vector<double>> m_mp;

	Notifier _notifer;

	Profiler(CANTalon & talon, std::vector<std::vector<double>> mp) : m_mp(mp), _talon(talon), _notifer(&Profiler::PeriodicTask, this)
	{
		/*
		 * since our MP is 10ms per point, set the control frame rate and the
		 * notifer to half that
		 */
		_talon.ChangeMotionControlFramePeriod(5);

		/* start our tasking */
		_notifer.StartPeriodic(0.005);
	}
	/**
	 * Called to clear Motion profile buffer and reset state info during
	 * disabled and when Talon is not in MP control mode.
	 */
	void reset()
	{
		_over = false;
		_talon.ClearMotionProfileTrajectories();
		/* When we do re-enter motionProfile control mode, stay disabled. */
		_setValue = CANTalon::SetValueMotionProfileDisable;
		/* When we do start running our state machine start at the beginning. */
		_state = 0;
		/*
		 * If application wanted to start an MP before, ignore and wait for next
		 * button press
		 */
		_bStart = false;
	}

	bool isOver() { return _over; }

	/**
	 * Called every loop.
	 */
	void control()
	{
		Timer controlBegin = Timer();
		controlBegin.Start();
		/* Get the motion profile status every loop */
		_talon.GetMotionProfileStatus(_status);

		/* first check if we are in MP mode */
		if(_talon.GetControlMode() != CANSpeedController::kMotionProfile){
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			_state = 0;
		} else { // MP Modes

			switch (_state) {
				case 0: /* wait for application to tell us to start an MP */
					if (_setValue == CANTalon::SetValueMotionProfileHold)
					{
							_over = true;
							std::cout << "MP ENDED!" << std::endl;
							_setValue = CANTalon::SetValueMotionProfileDisable;
					}
					if (_bStart) {
						_bStart = false;
						_over = false;
	
						_setValue = CANTalon::SetValueMotionProfileDisable;
						/*
						 * MP is being sent to CAN bus, wait a small amount of time
						 */
						_state = 1;
					}
					break;
				case 1: /*
						 * wait for MP to stream to Talon, really just the first few
						 * points
						 */
					/* do we have a minimum numberof points in Talon */
					if (_status.btmBufferCnt > kMinPointsInTalon) {
						/* start (once) the motion profile */
						_setValue = CANTalon::SetValueMotionProfileEnable;
						/* MP will start once the control frame gets scheduled */
						_state = 2;
					}
					break;
				case 2: /* check the status of the MP */
					if (_status.activePointValid && _status.activePoint.isLastPoint) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						_setValue = CANTalon::SetValueMotionProfileHold;
						_state = 0;
					}
					break;
			}
		}
		controlBegin.Stop();
		std::cout << "Time Elapsed for Control Period: " << controlBegin.Get() << std::endl;
	}

	/** Start filling the MPs to all of the involved Talons. */
	void startFilling()
	{
		/* since this example only has one talon, just update that one */
		startFilling(m_mp);
	}


	/* DEBUG CONVERSION THINGS */
	void startFilling(std::vector<std::vector<double>> profile)
	{
		std::cout << "Starting fill" << std::endl;

		int totalCnt = profile.size();
		/* create an empty point */
		CANTalon::TrajectoryPoint point;

		/* did we get an underrun condition since last time we checked ? */
		if(_status.hasUnderrun){
			_talon.ClearMotionProfileHasUnderrun();
		}

		_talon.ClearMotionProfileTrajectories();

		/* This is fast since it's just into our TOP buffer */
		for(int i=0;i<totalCnt;++i){
			/* for each point, fill our structure and pass it to API */
			point.position = profile[i][0] * 2.088; /*
											 * copy the position, velocity, and
											 * time from current row
											 */
			point.velocity = profile[i][1] * 125.287;
			point.timeDurMs = (int) profile[i][2] * 1000;
			point.profileSlotSelect = 1; /*
											 * which set of gains would you like to
											 * use?
											 */
			point.velocityOnly = false; /*
										 * set true to not do any position
										 * servo, just velocity feedforward
										 */

			point.zeroPos = false;
			if (i == 0)
				point.zeroPos = true; /* set this to true on the first point */

			point.isLastPoint = false;
			if( (i + 1) == totalCnt )
				point.isLastPoint = true; /*
											 * set this to true on the last point
											 */

			_talon.PushMotionProfileTrajectory(point);
		}
	}
	
	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	void start() {
		startFilling();
		_bStart = true;
	}

	/**
	 * 
	 * @return the output value to pass to Talon's set() routine. 0 for disable
	 *         motion-profile output, 1 for enable motion-profile, 2 for hold
	 *         current motion profile trajectory point.
	 */
	CANTalon::SetValueMotionProfile getSetValue() {
		return _setValue;
	}
};
#endif // MotionProfileExample__h_
