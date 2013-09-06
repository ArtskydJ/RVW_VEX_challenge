/* VARIABLES
 All variables for joystick sticks, buttons,
robot sensors, autonomous values, debugger
values, and general values.
*/
// SENSORS
int senAddToAbsGyro; //Not actual sensor...
int senLineL;
int senLineC;
int senLineR;
T_LC_INT senGyro;
T_LC_INT senAbsGyro;
T_LC_INT senUS;
T_LC_INT senQSEL;
T_LC_INT senQSER;
T_LC_INT senSlide;
T_LC_INT senAngle;

/* OUTPUT VARIABLES
 These variables are written-to, processed and
read-from, to help assign motor values.
*/
int outDrvL;
int outDrvR;
int outIntk;
int outLift;
int outAngl;
int outDump;

//Autonomous
int autoHitTarget;
int autoStepStatus;
bool autoFoundLeft;
bool autoFoundRight;
bool autoDriveReady;
bool autoLiftReady;
bool autoClockRunning = false;
int  autoStep = 0;
unsigned int autoTimer = 0;

//System
bool sysAutoMode = true; //Start in auton mode
bool sysDisabledMode = false;
T_LC_INT sysState;
int sysLooptime;

//PID
T_PID PIDAngle;
T_PID PIDSlide;
T_PID PIDLineFollow;
T_PID PIDDriveL;
T_PID PIDDriveR;
T_PID PIDGyro1;
T_PID PIDGyro2;

//--Arrays--//
short mtrTarget[10]={0,0,0,0,0,0,0,0,0,0};
short mtrSlewed[10]={0,0,0,0,0,0,0,0,0,0};
char slewConstants[2][10];//Intake,	Drive,	Reach 1,	2,	Goal,	Goal2,	Whackback,Catapult1,2,3
const int presetSlide[] = {-1115,	-1115,	-240,	-240,	-260,	-240,	-1115,	-240,	-240,	-240};
const int presetAngle[] = {325,		500,	500,	200,	1720,	1935,	2200,	320,	780,	2400};
unsigned int autoTimeRecord[NO_TIME_RECORDS];
