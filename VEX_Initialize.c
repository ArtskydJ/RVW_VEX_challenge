void initialize(int INinitPoint)
	{
	zeroMotors();
	//--SET VALUES--//
	//PID Controllers
	PIDAngle.kp = 0.8;
	PIDAngle.ki = 0.00;
	PIDAngle.kd = 0.00;

	PIDSlide.kp = 0.8;
	PIDSlide.ki = 0.00;
	PIDSlide.kd = 0.00;

	PIDLineFollow.kp = 0.2;
	PIDLineFollow.ki = 0.00;
	PIDLineFollow.kd = 0.00;

	PIDDriveL.kp = 0.8;
	PIDDriveL.ki = 0.00;
	PIDDriveL.kd = 0.00;

	PIDDriveR.kp = 0.8;
	PIDDriveR.ki = 0.00;
	PIDDriveR.kd = 0.00;

	PIDGyro1.kp = 1.8; //1 side
	PIDGyro1.ki = 0.00;
	PIDGyro1.kd = 0.02;

	PIDGyro2.kp = 1.2; //2 sides
	PIDGyro2.ki = 0.00;
	PIDGyro2.kd = 0.00;
	//Motor Slew Constants
	for (int j=0; j<10; j++)
		slewConstants[DISABLED][j]		= FULL;				//DISABLED
	slewConstants[AUTONOMOUS][DRIVE_L]	= AUTO_DRV_SLEW;	//AUTONOMOUS
	slewConstants[AUTONOMOUS][DRIVE_R]	= AUTO_DRV_SLEW;
	slewConstants[AUTONOMOUS][INTAKE]	= AUTO_INTK_SLEW;
	slewConstants[AUTONOMOUS][ANGLE]	= AUTO_ANGL_SLEW;
	slewConstants[AUTONOMOUS][SLIDE_L]	= AUTO_SLID_SLEW;
	slewConstants[AUTONOMOUS][SLIDE_R]	= AUTO_SLID_SLEW;
	slewConstants[AUTONOMOUS][TREAD]	= AUTO_TRED_SLEW;

	//--RESET TIMERS--//
	ClearTimer(T1); //Current Autonomous Step
	ClearTimer(T2); //PID wait
	ClearTimer(T3); //Entire Autonomous Time
	ClearTimer(T4); //unassigned

	//--MISCELLANEOUS--//
	writeDebugStreamLine("================");
	sysState.curr=AUTONOMOUS;
	setToZero(senSlide);
	setToZero(senQSEL);
	setToZero(senQSER);
	senAddToAbsGyro=INinitPoint;
	}
