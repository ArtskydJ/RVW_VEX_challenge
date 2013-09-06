void input()
	{
#ifdef PRACTICE //Practice
	if (sysDisabledMode)	sysState.curr = DISABLED;
	else					sysState.curr = AUTONOMOUS;
#else			//Competition
	if (bIfiRobotDisabled)	sysState.curr = DISABLED;
	else					sysState.curr = AUTONOMOUS;
#endif
	if (changed(sysState))
		{
		switch (sysState.curr)
			{
			case AUTONOMOUS: autoStep=0; break;
			case DISABLED: break;
			}
		}

	//--Robot Sensors--//
	senGyro.curr =		SensorValue[GYRO]%3600;
	senQSEL.curr =		SensorValue[QUAD_L];
	senQSER.curr =		SensorValue[QUAD_R];
	senUS.curr =		SensorValue[ULTRA];
	senSlide.curr =		-SensorValue[QUAD_SLIDE];
	senAngle.curr =		SensorValue[ANGLE_POT];
	senLineL =			SensorValue[LINE_L];
	senLineC =			SensorValue[LINE_C];
	senLineR =			SensorValue[LINE_R];

	if (autoClockRunning)
		autoTimer = time1(T3);

	if (senGyro.last>3400 && senGyro.curr<200) senAddToAbsGyro+=3600;
	if (senGyro.last<200 && senGyro.curr>3400) senAddToAbsGyro-=3600;
	senAbsGyro.curr = senAddToAbsGyro + senGyro.curr;
	}

////////////////////////////// I/O //////////////////////////////

void output()
	{
	if (sysState.curr == DISABLED)
		{
		zeroMotors();
		}
	else
		{
		mtrTarget[DRIVE_L] = outDrvL;
		mtrTarget[DRIVE_R] = outDrvR;
		mtrTarget[SLIDE_L] = outLift;
		mtrTarget[SLIDE_R] = outLift;
		mtrTarget[ANGLE]  = outAngl;
		mtrTarget[INTAKE] = outIntk;
		mtrTarget[TREAD] = outDump;
		}

	for (int j=0; j<10; j++)
		{
#ifdef SLEW
		mtrSlewed[j] += slew(mtrTarget[j], mtrSlewed[j], slewConstants[sysState.curr][j]); //SLEW CONTROLLERS
		capValue(-127, mtrSlewed[j], 127); //CAP ALL MOTORS
		motor[j] = mtrSlewed[j]; //ASSIGN MOTORS
#else
		capValue(-127, mtrTarget[j], 127); //CAP ALL MOTORS
		motor[j] = mtrTarget[j]; //ASSIGN MOTORS
#endif
		}
	string temp1,temp2;
	StringFormat(temp1,"Time:%.1f",((float)autoTimer/1000));
	StringFormat(temp2, "Step: %d", autoStep);	//Show step
	displayLCDCenteredString(0,temp1);
	displayLCDCenteredString(1,temp2);
	}
