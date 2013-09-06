//Definitions.c
//2013-05-06

/*FLAGS
 These flags are meant to be overall system
toggles that are easily turned off and on.
*/
#define AUTON_BEEP
#define SOUND_EFFECTS
//#define SLEW
//#undef _TARGET				//This statement and the next go together. Uncomment both
//#define _TARGET "VirtWorld"	//If you want the emulator to act like a virtual worlds robot.

/*FUNCTIONS
 These function-like defines are for shortening
and simplifying the code.
 The ones that have INLC as an input are for
last/curr structs (declared below).
 There are some for autonomous values so that
different sensors can be used.
 There are also constants for these functions.
*/
#define checkTarget(cndtn,value)	(cndtn)?((value)>=INtarget):((value)<=INtarget)
#define capValue(Min,Value,Max)		Value = (Value<Min)? (Min):(Value); Value = (Value>Max)? (Max):(Value)
#define changed(INLC)				(INLC.last != INLC.curr)
#define pressed(INLC)				(!INLC.last && INLC.curr)
#define diffLast(INLC)				(INLC.curr - INLC.last)
#define diffStep(INLC)				(INLC.curr - INLC.stepStart)
#define setLast(INLC)				INLC.last = INLC.curr
#define setStep(INLC)				INLC.stepStart = INLC.curr
#define setToZero(INLC)				INLC.last=0; INLC.curr=0; INLC.stepStart=0
#define fixIrregularity(INLC,n)		slew(INLC.curr,INLC.last,n)
#define K_STRAFE					10000
#define enc(n)						((K_STRAFE*0)+n)
#define L_US(n)						((K_STRAFE*1)+n)
#define R_US(n)						((K_STRAFE*2)+n)

/*Auton Routine Portions
 Portions of the auto routine.
*/
#define RUN_ALL		0
#define RUN_HALF	1
#define RUN_END		2

/*VALUES
 These values are stored here for easy modifica-
tion of system constants.
 NO means Number-Of.
*/
#define NO_AUTO_COLUMNS		7
#define NO_TIME_RECORDS		100

/*Time
 Minimum Loop time; scrolling text next
character; PID delay (for autonomous); LCD blink
time, slow and fast; LCD timeout (untouched).
*/
#define MIN_LOOP_MS			4
#define PID_WAIT_MS			0 //350

/*Preset Values
 Heights for lift, gyro, etc.
*/
#define L_INTK	1
#define L_DRIV	2
#define L_RCH1	3
#define L_RCH2	4
#define L_GOAL	5
#define L_GOA2	6
#define L_WHBK	7
#define L_CAT1	8
#define L_CAT2	9
#define L_CAT3	10

#define TURN_L	-900 //Relative-v-
#define TURN_R	900  //Relative-^-
#define BLUE_S	900  //Absolute-v-
#define RED_S	2700 //Absolute |
#define GOAL_Z	0    //Absolute |
#define HANG_Z	1800 //Absolute-^-

/*Drive Direction
 This is for which direction the robot is pointed
so that the drive can be translated correctly.
*/
#define DRV_FWD 0
#define DRV_LFT 1
#define DRV_REV 2
#define DRV_RHT 3

/*Motor Speed Constants
 Shortcuts for autonomous writing.
*/
#define UP		 127
#define DOWN	-127
#define FWD		 127
#define REV		-127
#define LEFT	-127
#define RIGHT	 127
#define BOTH	 0
#define IN		 127
#define OUT		-127
#define FULL	 127
#define HALF	 64
#define FOLLOW	 100
#define TURN	 127
#define BRAKE	 5 //can also be used in -Action- column

/*Motor Slew Constants
 Values for how much to add to each motor value
each loop iteration.
*/
#define AUTO_DRV_SLEW	3
#define AUTO_ANGL_SLEW	8
#define AUTO_INTK_SLEW	64
#define AUTO_SLID_SLEW	12
#define AUTO_TRED_SLEW	12

/*Robot States
 What state the robot is currently in...
*/
#define DISABLED	0
#define AUTONOMOUS	1


/*Hit Target
 These values are used in autonomous. If the end-
type is encoder, and the wheels have been driven
far enough, then the variable autoHitTarget is
set to NEXT. If the minimum time is higher than
the current time, it will not execute
autoNextStep() until it is.
 These values are also used as a trigger for a
delay to allow PID controllers to stablize before
the next step is executed.
*/
#define NOT_HIT	0
#define NEXT	1
#define PID		2

/*Next Condition
 These are for if the minimum or maximum time are
used in autonomous. If one is used, then a debug
stream message is sent to the debugger. In this
way, one can easily see which steps always time-
out.
*/
#define SENSOR_HIT	0
#define MIN_TIMEOUT	1
#define MAX_TIMEOUT	2

/*Line Following
 Edge, and line definitions
*/
#define LINE	1300

/*Autonomous End and Drive types
 The Drive types are for how the robot drives.
For example:
1 Line Follow
2 Gyro Strafe
3 Normal Drive
 The End types are for what sensor triggers the
step to end.
For example:
1 Left Encoder
2 Absolute Left Ultrasonic
3 Lift Potentiometer
*/
// End Types
#define RESET_AUTO	0	// Autonomous Done (End/Start of array)
#define TIME_LIMIT	1	// Time Limit
#define DRIV_READY	2	// Finished using Drive
#define LIFT_READY	3	// Finished using Lift
#define FULL_READY	4	// Finished using Drive and Lift
#define ONE_EDG_LN	5	// Cross Line
#define	TWO_EDG_LN	6	// Line up on white line
#define FRONT_LINE	7	// Center Line sensor hits
// Drive Types
#define SPD		0
#define ENC		1
#define STR		2
#define GYRO2	3
#define GYROL	4
#define GYROR	5
#define LINEC	6

/*Miscellaneous defined values
 The PID zone is so that small error values
owill not be ignored. This is so that the PID
controller can come close to the target, not
come to the exact value.
*/
#define PID_ZONE			25			//If the PID error is within this amount, it is close enough to its target

//--Typedefs--//
/*Structs
T means type
LC means Last/Current (see function-like defines
above; after flags, before values.)
PID means Proportional, Integral and Derivative.
*/
typedef struct
	{
	int curr;
	int last;
	int stepStart;
	}
	T_LC_INT;

typedef struct
	{
	float kp;
	float ki;
	float kd;
	int error;
	int integral;
	int derivative;
	int lastError;
	int output;
	}
	T_PID;
