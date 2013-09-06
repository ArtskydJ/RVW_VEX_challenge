void autoReset(int INcurrentStep)
	{
	zeroMotors();
	if (INcurrentStep == 0) //Runs at start of Autonomous
		{
		ClearTimer(T1);
		ClearTimer(T3);
		autoTimer=0;
		autoClockRunning = true;
		autoStep = 0;
		setToZero(senQSEL);
		setToZero(senQSER);
		setToZero(senSlide);
		}
	else //Runs at end of Autonomous
		{
		writeDebugStreamLine("----------------");
		writeDebugStreamLine("Time: %.1f",((float)autoTimer/1000));
		writeDebugStreamLine("----------------");
		autoClockRunning = false;
		setToZero(senQSEL);
		setToZero(senQSER);
		setToZero(senSlide);
		}
	}


void auto(int INdrvType, int INdrvLft, int INdrvRht, int INlift, int INdump, int INintake, int INendType, int INdelayPID)
	{

	//Next Step...
	string msg1,msg2="";
	if (autoStep < NO_TIME_RECORDS)
		autoTimeRecord[autoStep] = time1(T1);
	if (time1(T1)<1000)
		writeDebugStreamLine("%d\t\t|%d",time1(T1),autoStep);
	else
		writeDebugStreamLine("%d\t|%d",time1(T1),autoStep);
	autoClockRunning=true;
	autoFoundLeft = false;
	autoFoundRight = false;
	autoDriveReady = false;
	autoLiftReady = false;
	autoHitTarget = NOT_HIT;
	autoStepStatus = SENSOR_HIT;
	autoStep++;
	SensorValue[QUAD_L]=0;
	SensorValue[QUAD_R]=0;
	setToZero(senQSEL);
	setToZero(senQSER);

	//--Step start values--//
	setStep(senGyro);
	setStep(senAbsGyro);
	setStep(senQSEL);
	setStep(senQSER);
	setStep(senUS);
	setLast(senSlide);
	setLast(senAngle);
	ClearTimer(T1);
	/*while(time1(T1)<2000){}
	ClearTimer(T1);*/


	while(autoHitTarget!=NEXT)
		{
		input();

		//--Set Outputs--//
		outIntk = INintake;
		outDump = INdump;
		if (INlift==0)
			{
			outAngl = 0;
			outLift = 0;
			autoLiftReady = true;
			}
		else
			{
			outAngl = updatePIDController(PIDAngle, presetAngle[INlift-1] - senAngle.curr);
			outLift = updatePIDController(PIDSlide, presetSlide[INlift-1] - senSlide.curr);
			if (abs(PIDAngle.error) < PID_ZONE)	autoLiftReady = true;
			}


		/* Drive Outputs */
		outDrvL = 0;
		outDrvR = 0;
		switch (INdrvType)
			{
			case SPD:
				outDrvL = INdrvLft;
				outDrvR = INdrvRht;
				autoDriveReady = true;
				break;
			case ENC:
				updatePIDController(PIDDriveL,INdrvLft - diffStep(senQSEL));
				updatePIDController(PIDDriveR,INdrvRht - diffStep(senQSER));
				outDrvL = PIDDriveL.output;
				outDrvR = PIDDriveR.output;
				if (abs(PIDDriveL.error)<PID_ZONE && abs(PIDDriveR.error)<PID_ZONE) autoDriveReady = true;
				break;
			case STR:											//Gyro keep straight
				updatePIDController(PIDGyro2, INdrvLft-senAbsGyro.curr);
				//updatePIDController(PIDDriveL,INdrvRht - (diffStep(senQSEL) + diffStep(senQSER))/2 );
				updatePIDController(PIDDriveL,INdrvRht - diffStep(senQSEL));
				outDrvL = PIDDriveL.output;		//Left
				capValue(-TURN,outDrvL,TURN);
				outDrvL+=PIDGyro2.output;
				outDrvR = PIDDriveL.output;		//Right
				capValue(-TURN,outDrvR,TURN);
				outDrvR-=PIDGyro2.output;
				if (abs(PIDDriveL.error) < PID_ZONE && abs(PIDDriveR.error) < PID_ZONE) autoDriveReady = true;
				break;
			case LINEC:											//Follow Line
				updatePIDController(PIDLineFollow,senLineC-LINE);
				outDrvL = INdrvLft - PIDLineFollow.output;
				outDrvR = INdrvLft + PIDLineFollow.output;
				break;
			case GYRO2:											//Gyro turn both
				updatePIDController(PIDGyro2, INdrvLft-diffStep(senGyro));
				outDrvL =  PIDGyro2.output; capValue(-abs(INdrvRht),outDrvL,abs(INdrvRht));
				outDrvR = -PIDGyro2.output; capValue(-abs(INdrvRht),outDrvL,abs(INdrvRht));
				if (abs(PIDGyro2.error) < PID_ZONE) autoDriveReady = true;
				break;
			case GYROL:											//Gyro turn left - GOOD
				updatePIDController(PIDGyro1, INdrvLft-diffStep(senAbsGyro));
				outDrvL = PIDGyro1.output; capValue(-abs(INdrvRht),outDrvR,abs(INdrvRht));
				outDrvR = 0;
				if (abs(PIDGyro1.error) < PID_ZONE) autoDriveReady = true;
				break;
			case GYROR:											//Gyro turn right - GOOD
				updatePIDController(PIDGyro1, -INdrvLft+diffStep(senAbsGyro));
				outDrvL = 0;
				outDrvR = PIDGyro1.output; capValue(-abs(INdrvRht),outDrvR,abs(INdrvRht));
				if (abs(PIDGyro1.error) < PID_ZONE) autoDriveReady = true;
				break;
			}

		if (senLineL < LINE)	autoFoundLeft = true;	//Found Left Edge
		if (senLineR < LINE)	autoFoundRight = true;	//Found Right Edge
		if (INendType==TWO_EDG_LN)
			{
			if (autoFoundLeft)	outDrvL = BRAKE;
			if (autoFoundRight)	outDrvR = BRAKE;
			}

		if (autoHitTarget==NOT_HIT)
			{
			if (INendType!=RESET_AUTO && INendType!=TIME_LIMIT) //INdelayPID is used as time limit in TIME_LIMIT
				{
				if (INdelayPID<NEXT) INdelayPID=NEXT;
				if (INdelayPID>PID)  INdelayPID=PID;
				}
			ClearTimer(T2); //Timer for PID wait
			switch(INendType) // This code asks what type of target condition are we looking for? Have we met it?
				{
				case TIME_LIMIT: if (time1(T1)>=INdelayPID)				autoHitTarget=NEXT;		  break;
				case DRIV_READY: if (autoDriveReady)					autoHitTarget=INdelayPID; break;
				case LIFT_READY: if (autoLiftReady)						autoHitTarget=INdelayPID; break;
				case FULL_READY: if (autoDriveReady && autoLiftReady)	autoHitTarget=INdelayPID; break;
				case ONE_EDG_LN: if (autoFoundLeft || autoFoundRight)	autoHitTarget=INdelayPID; break;
				case TWO_EDG_LN: if (autoFoundLeft && autoFoundRight)	autoHitTarget=INdelayPID; break;
				case FRONT_LINE: if (senLineC<LINE)						autoHitTarget=INdelayPID; break;
				default: break; //nothing
				}
			}
		if (autoHitTarget==PID && time1(T2)>=PID_WAIT_MS)	autoHitTarget=NEXT; //PID timeout

		output();

		//--Sensors--//
		setLast(senGyro);
		setLast(senAbsGyro);
		setLast(senQSEL);
		setLast(senQSER);
		setLast(senUS);
		setLast(senSlide);
		setLast(senAngle);
		//--System--//
		setLast(sysState);

		sysLooptime=time1(T4);
		while (time1(T4) < MIN_LOOP_MS) {}
		ClearTimer(T4);
		}
	}