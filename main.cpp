/*
Virtual Solar Boat Motion Simulator Software V 0.1
By: Guillermo J Colom, mathematical models, transcription from Matlab
Co: Jezreel Saltar Vega, communication protocol and original arduino software

*/

#include <sys/ioctl.h>				//for SPI
#include <linux/spi/spidev.h>		//for SPI
#include <fcntl.h>					//for SPI
#include <cstring>					//for SPI
#include <unistd.h>					//for SPI
#include <iostream>					//general purpose
#include <stdio.h>					//general purpose
#include <stdlib.h>					//general purpose
#include <math.h>					//for math
#include <wiringPi.h>				//for timing

//+++++++++++++++++++++++++++++++Definitions++++++++++++++++++++++++++++++++++++++++++
//Program States
#define MENU 		0   //Table Paused, in Menu
#define SIMULATION  1   //Running in Simulation Mode
#define MANUAL      2   //Manual Input Mode
#define HOLD        3	//Hold mode

//Boat Model Definitions
#define Z_VAR       0   //Height
#define U_VAR       1   //Velocity in x
#define V_VAR       2   //Velocity in y
#define W_VAR       3   //Velocity in z
#define PHI_VAR     4   //Roll Angle
#define THETA_VAR   5   //Pitch Angle
#define PSI_VAR     6   //Yaw Angle
#define P_VAR       7   //Roll Velocity
#define Q_VAR       8   //Pitch Velocity
#define R_VAR       9   //Yaw Velocity
#define WR_VAR      10  //
#define II_VAR      11  //
#define BOAT_STATES 12	//Number of State Variables for Boat Model

#define U1 			0	//First Input
#define U2 			1	//First Input
#define U3 			2	//First Input
#define U4 			3	//First Input
#define BOAT_INPUTS 4	//Number of Boat Inputs

//----------------------------------------------------------------------------------------------
//Motion Table Definitions, Constants
//Dimension Constants, in mm
#define MOTION_W1  10.0   //Distance between radial point 1 and middle of both radial points of motion platform
#define MOTION_H1  89.85  //Perpendicular distance between center origin and radial point 1
#define MOTION_W2  10.0   //Distance between radial point 2 and middle of both radial points of motion platform
#define MOTION_H2  89.85  //Perpendicular distance between center origin and radial point 2
#define ALPHA_NUM  3      //Number of radial pairs
#define ROLL_MAX   30.0   //Abs bounds for max roll angle
#define PITCH_MAX  30.0   //Abs bounds for max pitch angle
#define YAW_MAX    25.0   //Abs bounds for max yaw angle
#define DELTAZ_MAX 40.0   //Abs bounds for max deltaZ
#define A_ARM      320.0  //Length of swinging arm
#define B_ARM      100.0  //Lingth of servo arm
#define BASE_SIDE  100.0  //BASE_SIDE
#define BASE_L1    97.85  //Perpendicular length from Base side at servo axises
#define BASE_L2    87.90  //Perpendicular length from Base side at servo axises

//Coordinate Array, Data index Positions
#define X_      0   //index for x coordinates
#define Y_      1   //index for y coordinates
#define Z_      2   //index for z coordinates
#define ROLL    0	//index for Roll in Input Array
#define PITCH   1	//index for Pitch in Input Array
#define YAW     2	//index for Yaw in Input Array
#define DELTAZ  3	//index for deltaZ in Input Array
#define THRUST  4	//index for thrust value, used only in simulation
#define KAL_X	5   //index for Kalman Angle x value, in SPI transaction
#define KAL_Y	6   //index for Kalman Angle y value, in SPI transaction
#define SERVO_INDEX_OFFSET 7 //Add to index for Servo Values
/*Reference Values
Pi -> M_PI

*/


//----------------------------------------------------------------------------------------------
//User Interface & Operation Mode Definitions
#define BOAT_CONTROLLER_INPUT 	0  	//Uses external boat controller to provide system state inputs
#define JOYSTICK_INPUT			1	//Uses onboard joysticks to provide system state inputs
#define MANUAL_ANGLE_HOLD		0	//Manual Operation mode for holding a stationary angle
#define MANUAL_JOYSTICK_INPUT	1	//Manual Operation mode for angles with joysticks

#define SLEEP 					10
#define DEEP_SLEEP				30
#define READING_RX				false //for debug printout set to "true"
#define CS0						10  //wiringPi CS0 pin Definition




//++++++++++++++++++++++++Variables & Functions+++++++++++++++++++++++++++++++++++++++

using namespace std;

//Program Control Variables
int programState = 0;														//0 Menu, 1 Simulation Mode, 2 Manual Control Mode
bool exitCondition = false;													//Condition for Quiting the Program
double samplePeriod = 0.005;													//Sampling Rate or 5ms, 200Hz

//Boat Model Variables
double x_0[BOAT_STATES] = {-0.3048000, 8.62, 0, 0, 0, 0, 0, 0, 0, 0, 130, 275}; 	//Vector of Initial State Values
double xState[BOAT_STATES] = {-0.3048000, 8.62, 0, 0, 0, 0, 0, 0, 0, 0, 130, 275}; 	//Vector of Current State Values, initialized to initial Values
double xDot[BOAT_STATES];													//Will Hold Numeric Dot Product of Boat States
double uInputs[4] = {0.02, 0.02, 0.02, 0.02};								//Input Vector, holds current inputs, initialized at 0.02
double alpha = 0;															//Some Angular value
double mass = 226.796; 														//mass of boat in kg
double rho = 1000.0;														//Water Density
double Vmotor = 24.0;														//motorVoltage
double BDamping = 0.0;														//Sleeve Dampening
double JInertia = .25;													//Sleeve Inertia
double RArm, Kmotor, LArm = 0.01;											//Motor Armature Resistance, Motor Voltage Constant, Armature Inductance
double io;																	//Correction current (paper mit permanent magnet motor and prop)
double Ia, Ea, Pd, Tm;														//Poorly Labeled variables belonging to the inner processes of the Boat Model
double rf1x = 0.8128, rf1y = 0.6096, rf1z = 0.9144;							//Foil Radius
double rf2x, rf2y, rf2z;													//Foil Radius
double rf3x = 1.6256, rf3y = 0, rf3z;										//Foil Radius
double rfsx, rfsy, rfsz;													//Foil Radius
double rfi1x, rfi1y, rfi1z, rfi2x, rfi2y, rfi2z, rfi3x, rfi3y, rfi3z;		//Fin Radii
double af1 = 0.092903, af2 = 0.092903, af3 = 0.092903, afs = 0.05419344;	//Foil Areas: Right, Left, Back, back static
double afi1 = 0.06387084, afi2 = 0.06387084, afi3 = 0.0962498075;			//Fin Areas: Left, Right, Back... values when submerged by 1ft
double angf1, angf2, angf3, angfs;											//Fin angles, relative boat angle
double cdf1, cdf2, cdf3, cdfs;												//Foil Drag Polynomial Results
double clf1, clf2, clf3, clfs;												//Foil Lift Polynomial Results
double fdf1, fdf2, fdf3, fdfs;												//Foil Drag Forces
double fdfi1, fdfi2, fdfi3;													//Fin Drag Forces
double drag;																//Total Drag
double flf1, flf2, flf3, flfs, flfi1, flfi2, flfi3;							//Foil Lift Forces (first four), Fin Lift Forces (last three)
double liftF, liftFi;														//Total Lift of foils and fins, respectively
double rX[3][3], rY[3][3], rZ[3][3],rZY[3][3], rbi[3][3], rib[3][3];		//3x3 matrices for rotational values
double TL, fm, FD;															//Shaft Torque, prop force, hull drag
double vectorX[3] = {1, 0, 0};												//Vector for x values
double vectorY[3] = {0, 1, 0};												//Vector for Y values
double vectorZ[3] = {0, 0, 1};												//Vector for Z values
double vectorGrav[3];														//Vector for Weight, must be initialized
//double Ixx = 351.95, Iyy = 25, Izz = 50;									//Obsolete Inertial Values
double Ixx = 0.06584, Iyy = 351.1532, Izz = 354.707;						//Inertial Values


//Boat Model Functions
void boatModelInitialize();													//function to initialize static value variables that cannot be declared
void boatModelUpdate();
//void boatModelInputsUpdate();
double ee(int power);														//Returns Powers of 10
//Rotation Matrix Operations
void rotationX(double phi);													//populates matrix rX
void rotationY(double theta);												//populates matrix rY
void rotationZ(double psi);													//populates matrix rZ
void matrixMultiply(double mat1[3][3], double mat2[3][3], double res[3][3]);//Multiply two Matrices mat1*mat2
void matrixInverse(double mat[3][3], double inv[3][3]);						//Inverts Matrix
double vectorMatrixVector(double v1[3], double mat[3][3], double v2[3]);	//vector*matrix*vector
//Print State Array
void printVector(double vec[BOAT_STATES], int states);						//Print State Variables
void printMatrixD(double arr[3][3]);										//Print Matrix

//----------------------------------------------------------------------------------------------
//Motion Table Global Variables
//Single Initialization
double  baseHalf;            //half of BASE_SIDE, initialized in setup()
double  degreeToRadian;      //Constant to convert degrees to radians, initialized in setup()
double  radianToDegree;      //Constant to convert radians to degrees, initialized in setup()
double  zBase;               //vertical distance when servos are at zero
double  alpha_degrees[ALPHA_NUM] = {90., 210., 330.};
              //Primary angle Degrees of radial point pairs
double  alpha_radians[ALPHA_NUM];
              //Primary angle Degrees of radial point pairs, initialized in setup()
double  rPoints[3][2][3];    //"Zeroed" coordinates of radial points, initialized in setup()
double  rPointsAdj[3][2][3]; //Points adjusted for rotation
double  beta1;               //Angle from alpha that radial point 1 deviates from centerline
double  beta2;               //Angle from alpha that radial point 2 deviate2 from centerline
double  r1Magnitude;          //Abs distance from origin to radial point 1
double  r2Magnitude;          //Abs distance from origin to radial point 2
double  sqrBArm;             // Square of B_Arm
double  sqrAArm;             // Square of A_Arm
//Dynamic Values
double  rollInput   = 0;     //Roll value reference
double  pitchInput  = 0;     //Pitch value reference
double  yawInput    = 0;     //Yaw value reference
double  deltaZInput = 0;     //DeltaZ value reference
double 	thrusterInput = 0;
double  rollOutput;          //Roll value to be used for calculations
double  rollOutputDeg;       //Roll value, in degrees
double  pitchOutput;         //Pitch value to be used for calculations
double  pitchOutputDeg;      //Pitch value, in degrees
double  yawOutput;           //Yaw value to be used for calculations
double  deltaZOutput;        //deltaZ value to be used for calculations
double  cRoll;               //cosine of roll
double  sRoll;               //sine of roll
double  cPitch;              //cosine of pitch
double  sPitch;              //sine of pitch
double  cYaw;                //cosine of yaw
double  sYaw;                //sine of yaw
double  cAlphaYaw[3];   //cosine of alpha yaw difference
double  sAlphaYaw[3];   //sine of alpha yaw difference
double  rotationA[3];   //constant value calculated from arithmetic with sines and cosines, 1 per alpha
double  rotationB[3];   //constant value calculated from arithmetic with sines and cosines, 1 per alpha
double  rotationC[3];   //constant value calculated from arithmetic with sines and cosines, 1 per alpha
double  rotationD[3];   //constant value calculated from arithmetic with sines and cosines, 1 per alpha
double  rotationE;      //constant value calculated from arithmetic with sines and cosines, 1 per alpha
double  rotationF;      //constant value calculated from arithmetic with sines and cosines, 1 per alpha
double  servoGammas[7];   //ServoAngles, in degrees
//double  previousServoGammas[7] = {0,0,0,0,0,0,0};   //ServoAngles of previous iteration, in degrees, initialize at 0 each
double  servoSlopes[7];   //Slopes of linearizations for servo angle to microseconds
int     microPairsSlope[7][2]={{1509,1817},{1467,1768},{1524,1812},{1554,1840},{1449,1735},{1541,1849},{1428,1694}};
              //Pairs of microsecond, difference used to calculate slope
              //First member of pairs is also the intercept "b" in ax+b
int   	angleSlope[7]={-30,-30,-30,-30,-30,-30,-30};
              //angles to be used to calculate slope. The other angle is 90. For servos 11,12,21,22,31,32,yaw respectively
float   servoValuesCurrent[7] = {1500, 1500, 1500, 1500, 1500, 1500,1500};			//holds values of Servos
float 	servoZeroValues[7];

float   rawInputs[5] = {0,0,0,0,0};			//holds integer analog values for manual/simmulation inputs/references
double 	conditionedInputs[4];				//holds values of reference values for motion table

//Input Variables
int rollInDir = -1;
int pitchInDir = 1;
int yawInDir = -1;
int deltaZInDir = 1;
double rollInputSlope;
double pitchInputSlope;
double yawInputSlope;
double deltaZInputSlope;
double rollInputIntercept;
double pitchInputIntercept;
double yawInputIntercept;
double deltaZInputIntercept;
//Input Filter Globals
double inputMean[4] = {0,0,0,0};		//Holds means for input Filter
int span = 10;

//PID Controller Variables
double roll_error_prior = 0;  //previous diff between ref and actual roll
double roll_integral = 0;   //previous roll integral
double roll_error;
double roll_derivative;

double pitch_error_prior = 0; //previous diff between ref and actual pitch
double pitch_integral = 0;    //previous pitch integral
double pitch_error;
double pitch_derivative;

double Kp = 0.5;       //proportional constant
double Ki = 4.5;        //integral constant
double Kd = 0.000250;        //derivative constant

//Tilt Protection Variables
bool tilt = false;
int tiltCountdown = 0;
int tiltCount = 50;

//Kalman Variables
double kalAngle[2] = {0,0}; // Calculated angle using a Kalman filter, X_ is pitch, Y_ is roll
double dt;

//Motion Table Functions
double 	square(double x);		//square two numbers
void 	motionTableInitialize();	//initialize non-dynamic variables of motion table
void 	motionTableUpdate();		//update servo values for motion table
double 	inputFilter(int index, double input);	//Filter Input
double 	yawInputCorrection(int inputRead);    //Corrects errors of yaw potentiometer


//--------------------------------------------------------------------------------------------
//Menu System, Timing, & Control
unsigned int startTime;                         //For the timing of processes
unsigned int microsDuration;                   //For the timing of processes
char userInput;									//holds last input put in by user
char menusSelectionValidIns[6] = {'1','2','3','4','5','6'};	//List of valid Inputs for Menu
bool validInput;
int samplePeriodMicros;							//length of sampling period in microseconds, initialized in motionTableInitialize()
int menuPage = 0;								//Menu Page
int inputMode = 0;								//mode with which manual and simulation acquires input information
double motionDurationMillis = 10000;			//amount of time to run motion table
int simulationInputType = 0;					//Simulation input type selection
int manualModeType = 0;							//Manual Mode type selection
double holdAngles[4] = {0,0,0,0};				//roll, pitch, yaw, and deltaZ for angle hold
bool dataLoggingEnabled = false;				//data logging toggle
double maximumDuration = 300000;				//5 minutes in milliseconds


//User interface Functions
void 	userInterface();						//Menu Interface for user
int 	userInputToInt(char input);				//convert char intput into int, after validation
bool 	inputValid(char valid[],int validSize, char input);
void 	menuStartText();						//Display Main Menu Page
void 	menuSimulationText();					//Display Simulation Menu Page
void 	menuManualModeText();					//Display Manual Mode Menu Page
void 	runMotionTable();						//Send and Receive Data between RPi and Arduino, using update functions, do the thing
void 	menuSettingsText();						//display text for settings menu

//--------------------------------------------------------------------------------------------
//SPI Communications
char receivedByte;          //Byte received from slave
bool ack;                   //boolean to acknowledge if slave returned handshake
int fd;                     //For SPI commands
union one  { float f; char  b[4]; } dataToTXRX[30];
//unsigned int sleep = 20;				//delay between SPI events

//Functions
void spiInitializatize();	//Initialize SPI functionality
void spiTransaction();		//Carry out exchange of data, once per cycle
int spiTxRx(char txDat); 	//SPI function to Transmit and Receive
float receive();         	//Function to Receive a float
float sendpwm(float servoPWM[7]);
void spiExchange();			//function to exchange bytes


int main()
{
	//Setup
	wiringPiSetup();                             //For timer
	pinMode(CS0, OUTPUT);						 //set slave select to output
	digitalWrite(CS0, HIGH);
	boatModelInitialize();                       //Initialize non-dynamic boat variables
	motionTableInitialize();                     //Initialize non-dynamic motion table variables
	spiInitializatize();						 //Initialize SPI functionality
    userInterface();							 //Menu System for starting operations
    system("clear");
    return 0;
}//end main()

//Function to Initialize Variables for Boat Model that require operations to initialize
void boatModelInitialize()
{
	RArm = 1.95*.012;														//Armature Resistance Value
	Kmotor = 71*2*M_PI/60;													//Motor Voltage Constant
	io = 3.5 + (Vmotor - 12)/12;											//Correction Current
	rf2x = rf1x;
	rf2y = rf1y;
	rf2z = rf1z;
	rf3z = rf2z;
	rfsx = rf3x;
	rfsy = rf3y;
	rfsz = rf3z;
	rfi1x = rf1x;
	rfi1y = rf1y;
	rfi1z = rf1z*(2/3);
	rfi2x = rf2x;
	rfi2y = rf2y;
	rfi2z = rf2z*(2/3);
	rfi3x = rf3x;
	rfi3y = rf3y;
	rfi3z = rf3z*(2/3);
	vectorGrav[0] = 0;
	vectorGrav[1] = 0;
	vectorGrav[2] = mass*9.81;
}// end boatModelInitialize()

//Function to be called once per simulation cycle
void boatModelUpdate()
{
	//The Following Equations were taken from original MatLab Source
	Ia = xState[II_VAR] - io;
	Ea = xState[WR_VAR]/Kmotor;
	Pd = Ea*Ia;
	Tm = Pd/xState[WR_VAR];
	//Update Fin Angles
	angf1 = xState[THETA_VAR];
	angf2 = xState[THETA_VAR];
	angf3 = xState[THETA_VAR];
	angfs = xState[THETA_VAR];
	//Foil Drag polynomials
	cdf1 = -7.2*ee(-6)*(pow((uInputs[U1] + angf1)*180/M_PI,4)) +
		3.4*ee(-20)*(pow(((uInputs[U1] + angf1)*180/M_PI),3)) +
		0.0012*(pow(((uInputs[U1] + angf1)*180/M_PI),2)) -
		2*ee(-19)*((uInputs[U1] + angf1)*180/M_PI) + 0.0068;
	cdf2 = -7.2*ee(-6)*(pow((uInputs[U2] + angf2)*180/M_PI,4)) +
		3.4*ee(-20)*(pow(((uInputs[U2] + angf2)*180/M_PI),3)) +
		0.0012*(pow(((uInputs[U2] + angf2)*180/M_PI),2)) -
		2*ee(-19)*((uInputs[U2] + angf2)*180/M_PI) + 0.0068;
	cdf3 = -7.2*ee(-6)*(pow((uInputs[U3] + angf3)*180/M_PI,4)) +
		3.4*ee(-20)*(pow(((uInputs[U3] + angf3)*180/M_PI),3)) +
		0.0012*(pow(((uInputs[U3] + angf3)*180/M_PI),2)) -
		2*ee(-19)*((uInputs[U3] + angf3)*180/M_PI) + 0.0068;
	cdfs = -7.2*ee(-6)*(pow(angfs*180/M_PI,4)) +
		3.4*ee(-20)*(pow((angfs*180/M_PI),3)) +
		0.0012*(pow((angfs*180/M_PI),2)) -
		2*ee(-19)*(angfs*180/M_PI) + 0.0068;
	//printf("%f %f %f %f \n",cdf1, cdf2, cdf3, cdfs );   //For Debugging

	//Foil Lift Polynomials
	clf1 = 0.9554*pow((uInputs[U1] + angf1),2) + 4.2468*(uInputs[U1] + angf1);
	clf2 = 0.9554*pow((uInputs[U2] + angf2),2) + 4.2468*(uInputs[U2] + angf2);
	clf3 = 0.9554*pow((uInputs[U3] + angf3),2) + 4.2468*(uInputs[U3] + angf3);
	clfs = 0.9554*pow((angfs),2) + 4.2468*(angfs);
	//printf("%f %f %f %f \n",clf1, clf2, clf3, clfs );   //For Debugging

	//Foil Drag Forces
	fdf1 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rf1z - xState[R_VAR]*rf1y),2)*(af1*cdf1)*4.45;
	fdf2 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rf2z - xState[R_VAR]*rf2y),2)*(af2*cdf2)*4.45;
	fdf3 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rf3z - xState[R_VAR]*rf3y),2)*(af3*cdf3)*4.45;
	fdfs = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rfsz - xState[R_VAR]*rfsy),2)*(afs*cdfs)*4.45;
	//printf("%f %f %f %f \n",fdf1, fdf2, fdf3, fdfs );   //For Debugging

	//Fin Drag Forces
	fdfi1 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rfi1z - xState[R_VAR]*rfi1y),2)*afi1*0.005*4.45;
	fdfi2 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rfi2z - xState[R_VAR]*rfi2y),2)*afi2*0.005*4.45;
	fdfi3 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rfi3z - xState[R_VAR]*rfi3y),2)*afi3*0.005*4.45;
	//printf("%f %f %f \n",fdfi1, fdfi2, fdfi3);   //For Debugging

	//Total Drag
	drag = -fdf1 - fdf2 - fdf3 - fdfs - fdfi1 - fdfi2 - fdfi3;
	//printf("%f \n", drag);   //For Debugging

	//Foil Lift Forces
	flf1 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rf1z - xState[R_VAR]*rf1y),2)*af1*
	   clf1*(xState[Z_VAR] < 0)*4.45;
   	flf2 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rf2z - xState[R_VAR]*rf2y),2)*af2*
	   clf2*(xState[Z_VAR] < 0)*4.45;
	flf3 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rf3z - xState[R_VAR]*rf3y),2)*af3*
	   clf3*(xState[Z_VAR] < 0)*4.45;
	flfs = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rfsz - xState[R_VAR]*rfsy),2)*afs*
	   clfs*(xState[Z_VAR] < 0)*4.45;
	//printf("%f %f %f %f \n",flf1, flf2, flf3, flfs );   //For Debugging

	//Fin Lift Force
	fdfi1 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rfi1z - xState[R_VAR]*rfi1y),2)*afi1*0.005*4.45;
	fdfi2 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rfi2z - xState[R_VAR]*rfi2y),2)*afi2*0.005*4.45;
	fdfi3 = 0.5*rho*pow((xState[U_VAR] + xState[Q_VAR]*rfi3z - xState[R_VAR]*rfi3y),2)*afi3*0.005*4.45;
	//printf("%f %f %f \n",flfi1, flfi2, flfi3 );   //For Debugging

	//Total Foil Lift
	liftF = -flf1 - flf2 - flf3 - flfs;
	//printf("%f \n", liftF);   //For Debugging

	//Total Fin Lift
	liftFi = -flfi1 - flfi2 - flfi3;
	//printf("%f \n", liftFi);   //For Debugging

	//Rotational Matrices
	rotationX(xState[PHI_VAR]);				//populates matrix rX
	//printMatrixD(rX);
	rotationY(xState[THETA_VAR]);			//populates matrix rY
	//printMatrixD(rY);
	rotationZ(xState[PSI_VAR]);				//populates matrix rZ
	//printMatrixD(rZ);
	matrixMultiply(rZ,rY,rZY);				//multiplies rZ & rY
	//printMatrixD(rZY);
	matrixMultiply(rZY, rX, rbi);			//multiplies rZY & rX
	//printMatrixD(rbi);
	matrixInverse(rbi,rib);					//invert matrix
	//printMatrixD(rib);

	//Shaft Prop Torque
	TL = 4.521 - 7.508*xState[U_VAR] + 0.1437*xState[WR_VAR] + 0.3741 * pow(xState[U_VAR],2) +
		+ 0.00442*xState[U_VAR]*xState[WR_VAR] + 0.002596*pow(xState[WR_VAR],2) - 0.04312*pow(xState[U_VAR],3) +
		0.002757*pow(xState[U_VAR],2)*xState[WR_VAR] - 5.879*ee(-05)*xState[U_VAR]*pow(xState[WR_VAR],2) -
		1.399*ee(-19)*pow(xState[WR_VAR],3);
	//printf("%f \n", TL);   //For Debugging

	//Prop Thrust Force
	fm = 68.82 - 1.247*xState[U_VAR] - 1.513*xState[WR_VAR] - 7.309*pow(xState[U_VAR],2) + 0.631*xState[U_VAR]*xState[WR_VAR] +
		0.02599*pow(xState[WR_VAR],2) + 0.2337*pow(xState[U_VAR],3) - 0.05057*pow(xState[U_VAR],2)*xState[WR_VAR] +
		0.001882*xState[U_VAR]*pow(xState[WR_VAR],2) + 2.524*ee(-18)*pow(xState[WR_VAR],3);
	//printf("%f \n", fm);   //For Debugging

	//Hull Drag Force
	FD = (14.573*pow(xState[U_VAR],2) + 19.919*xState[U_VAR] + 6.385);
	//printf("%f \n", FD);   //For Debugging

	//Dot Functions
	xDot[WR_VAR] = ((Tm - TL/0.88) - BDamping*xState[WR_VAR])*(1/JInertia);
	xDot[II_VAR] = (Vmotor - xState[II_VAR]*RArm - Ea)*(1/LArm);
	xDot[U_VAR]	 = (xState[R_VAR]*xState[V_VAR] - xState[Q_VAR]*xState[W_VAR]) + (1/mass)*(drag +
   		fm + vectorMatrixVector(vectorX, rib, vectorGrav));
	//printf("%f ",vectorMatrixVector(vectorX, rib, vectorGrav));					//Error Checking
	xDot[V_VAR]  = (xState[P_VAR]*xState[W_VAR] - xState[R_VAR]*xState[U_VAR]) + (1/mass)*(liftFi +
		vectorMatrixVector(vectorY, rib, vectorGrav));
	//printf("%f 3",vectorMatrixVector(vectorY, rib, vectorGrav));					//Error Checking
	xDot[W_VAR]  = (xState[Q_VAR]*xState[U_VAR] - xState[P_VAR]*xState[V_VAR]) + (1/mass)*(liftF +
		vectorMatrixVector(vectorZ, rib, vectorGrav));
	//printf("%f \n",vectorMatrixVector(vectorZ, rib, vectorGrav));					//Error Checking
	xDot[P_VAR]  = 1/Ixx*(-rf1y*flf1 + rf2y*flf2 + rf3z*fm*sin(alpha) - rfi1z*flfi1 -
	    rfi2z*flfi2 - rfi3z*(flfi3*cos(alpha) + fdfi3*sin(alpha)) - xState[Q_VAR]*xState[R_VAR]*(Izz - Iyy));
	xDot[Q_VAR]  = 1/Iyy*(-rf1z*fdf1 - rf2z*fdf2 - rf3z*fdf3 - rfsz*fdfs + rf3z*fm*cos(alpha) +
		rf1x*flf1 + rf2x*flf2 - rf3x*flf3 - rfsx*flfs - rfi1z*fdfi1 - rfi2z*fdfi2 - rfi3z*(fdfi3*cos(alpha) -
		flfi3*sin(alpha)) - xState[P_VAR]*xState[R_VAR]*(Ixx - Izz));
	xDot[R_VAR]  = 0/Izz*(rf1y*fdf1 - rf2y*fdf2 - rf3x*fm*sin(alpha) + rfi1y*fdfi1 - rfi2y*fdfi2 -
		rfi3x*(flfi3*cos(alpha) - fdfi3*sin(alpha)) + rfi1x*flfi1 + rfi2x*flfi2 -
		xState[P_VAR]*xState[Q_VAR]*(Iyy - Ixx));
	xDot[Z_VAR] = xState[W_VAR];
	xDot[PHI_VAR] = xState[P_VAR];
	xDot[THETA_VAR] = xState[Q_VAR];
	xDot[PSI_VAR] = xState[R_VAR];

	//printVector(xDot, BOAT_STATES);			//For Debugging
	//Update xState
	for(int i = 0; i < BOAT_STATES; i++)
	{
		xState[i] += dt*xDot[i];
	}// end update for

}// end boatModelUpdate()

//Function for exponents of ten
double ee(int power)
{
	//uses pow from math.h
	return pow(10, power);
}//end EE()

//Generate X Rotation Matrix
void rotationX(double phi)
{
	rX[0][0] = 1;
	rX[0][1] = 0;
	rX[0][2] = 0;
	rX[1][0] = 0;
	rX[1][1] = cos(phi);
	rX[1][2] = -sin(phi);
	rX[2][0] = 0;
	rX[2][1] = sin(phi);
	rX[2][2] = cos(phi);
}//end rotationX()

//Generate Y Rotation Matrix
void rotationY(double theta)
{
	rY[0][0] = cos(theta);
	rY[0][1] = 0;
	rY[0][2] = sin(theta);
	rY[1][0] = 0;
	rY[1][1] = 1;
	rY[1][2] = 0;
	rY[2][0] = -sin(theta);
	rY[2][1] = 0;
	rY[2][2] = cos(theta);
}//end rotationY()

//Generate Z Rotation Matrix
void rotationZ(double psi)
{
	rZ[0][0] = cos(psi);
	rZ[0][1] = -sin(psi);
	rZ[0][2] = 0;
	rZ[1][0] = sin(psi);
	rZ[1][1] = cos(psi);
	rZ[1][2] = 0;
	rZ[2][0] = 0;
	rZ[2][1] = 0;
	rZ[2][2] = 1;
}//end rotationZ()

//Multiply two Matrices
void matrixMultiply(double mat1[3][3], double mat2[3][3], double res[3][3])
{
	for(int i = 0; i < 3; i++)
	{
		for(int n = 0; n < 3; n++)
		{
			res[i][n] = 0;
			for (int t = 0; t < 3; t++)
			{
				res[i][n] = res[i][n] + mat1[i][t]*mat2[t][n];
			} //end for sums
		}//end for columns
	}//end for rows
}//end matrixMultiply()

//Invert Matrix and store in new matrix
void matrixInverse(double mat[3][3], double inv[3][3])
{
	double det;
	det = -mat[0][2]*mat[1][1]*mat[2][0] + mat[0][1]*mat[1][2]*mat[2][0] + mat[0][2]*mat[1][0]*mat[2][1] -
		mat[0][0]*mat[1][2]*mat[2][1] - mat[0][1]*mat[1][0]*mat[2][2] + mat[0][0]*mat[1][1]*mat[2][2];
	for (int i = 0; i < 3; i++)
	{
		for (int n = 0; n < 3; n++)
		{
			inv[n][i] = (mat[(i+1)%3][(n+1)%3]*mat[(i+2)%3][(n+2)%3] - mat[(i+2)%3][(n+1)%3]*mat[(i+1)%3][(n+2)%3])/det;
		}// end for column
	}//end for row
}//end matrixInverse()

//Multiply Vector, matrix, and vector to get single value
double vectorMatrixVector(double v1[3], double mat[3][3], double v2[3])
{
	double temp[3] = {0,0,0};
	double result = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int t = 0; t < 3; t++)
		{
			temp[i] = temp[i] + v1[t]*mat[t][i];
		}//end for
	}//end for
	for(int i = 0; i < 3; i++)
	{
		result = result + temp[i]*v2[i];
	}//end for
	return result;
}//end vectorMatrixVector()

//Print out All of the State Variables
void printVector(double vec[BOAT_STATES], int states)
{
	for(int i = 0; i < states; i++)
	{
		printf("%f ", vec[i]);
		if (i == states - 1) printf("\n");
	}// end print for
}//end printState()


void printMatrixD(double arr[3][3])
{
    for(int i = 0; i < 3; i++)
    {
        for(int n = 0; n < 3; n++)
        {
            printf("%f ",arr[i][n]);
            if (n == 2) printf("\n");
        }
    }
    printf("\n");
}//end printMatrixD()

//-------------------------------------------------------------------------------------------------------------
//Motion Table Functions

//return square value double
double square(double x)
{
	//Pretty self-explanatory
	return x*x;
}//end square()

//Initialize non-dynamic variables for motion table
void motionTableInitialize()
{
	//One-time conversion initialized
	samplePeriodMicros = samplePeriod*1000000;
    baseHalf = (double)BASE_SIDE/2.0;
    degreeToRadian = M_PI/180.0;
    radianToDegree = 180.0/M_PI;
    zBase = sqrt(square(A_ARM)-square(B_ARM+baseHalf-MOTION_W2));
    beta1 = atanf(MOTION_W1/MOTION_H1);
    beta2 = atanf(MOTION_W2/MOTION_H2);
    r1Magnitude = sqrt(square(MOTION_W1)+square(MOTION_H1));
    r2Magnitude = sqrt(square(MOTION_W2)+square(MOTION_H2));
    sqrBArm = square(B_ARM);
    sqrAArm = square(A_ARM);

    rollInputSlope = (double)rollInDir*(ROLL_MAX/512.0);
    rollInputIntercept = -512.0*rollInputSlope;
    pitchInputSlope = (double)pitchInDir*(PITCH_MAX/512.0);
    pitchInputIntercept = -512.0*pitchInputSlope;
    yawInputSlope = (double)yawInDir*(YAW_MAX/512.0);
    yawInputIntercept = -512.0*yawInputSlope;
    deltaZInputSlope = (double)deltaZInDir*(DELTAZ_MAX/512.0);
    deltaZInputIntercept = -512.0*deltaZInputSlope;
    dt = samplePeriod;

    for (int i = 0; i < ALPHA_NUM; i++)
    {

		//Serial.println(alpha_degrees[i]);
		//Serial.print(" ");

		alpha_radians[i]    = alpha_degrees[i]*degreeToRadian;

		//Serial.println(alpha_radians[i]);

		rPoints[i][0][X_]  = r1Magnitude*cosf(alpha_radians[i]+beta1);
		rPoints[i][1][X_]  = r2Magnitude*cosf(alpha_radians[i]-beta2);
		rPoints[i][0][Y_]  = r1Magnitude*sinf(alpha_radians[i]+beta1);
		rPoints[i][1][Y_]  = r2Magnitude*sinf(alpha_radians[i]-beta2);
		rPoints[i][0][Z_]  = 0.0;
		rPoints[i][1][Z_]  = 0.0;
    } //end for
    //Servo Initialization, 7 times
    for (int i = 0; i < 7; i++)
    {
		servoSlopes[i]    = (double)(microPairsSlope[i][0]-microPairsSlope[i][1])/(double)(0 - angleSlope[i]);
		servoValuesCurrent[i] = (0 + microPairsSlope[i][0]);//zero the servos
		servoZeroValues[i] = (0 + microPairsSlope[i][0]);//zero the servos
		#if READING_RX
		printf("%f \n", servoValuesCurrent[i]);
		#endif
    }//end for

}//end motionTableInitialize()

//Update Servo values for motion table for either operation type
void motionTableUpdate()
{
	yawOutput     = conditionedInputs[YAW]*degreeToRadian;
    deltaZOutput  = conditionedInputs[DELTAZ];


	//PID Controllers

	rollInput = conditionedInputs[ROLL];
	roll_error = rollInput - kalAngle[Y_];
	roll_integral = roll_integral + (roll_error*dt);
	roll_derivative = (roll_error - roll_error_prior)/dt;
	rollOutputDeg = (Kp*roll_error + Ki*roll_integral + Kd*roll_derivative);
	rollOutput = degreeToRadian*rollOutputDeg;
	roll_error_prior = roll_error;
	//printf("%f\n", rollOutputDeg);


	pitchInput = conditionedInputs[PITCH];
	pitch_error = pitchInput - kalAngle[X_];
	pitch_integral = pitch_integral + (pitch_error*dt);
	pitch_derivative = (pitch_error - pitch_error_prior)/dt;
	pitchOutputDeg = (Kp*pitch_error+Ki*pitch_integral+Kd*pitch_derivative);
	pitchOutput = degreeToRadian*pitchOutputDeg;
	pitch_error_prior = pitch_error;
	//printf("%f\n", pitchOutputDeg);


	//Tilt Protection
	if (abs(rollOutputDeg) > 3*ROLL_MAX || abs(pitchOutputDeg) > 3*PITCH_MAX ||
		abs(deltaZOutput) > DELTAZ_MAX || abs(yawOutput) > YAW_MAX)
	{
		if (!tilt) tiltCountdown = tiltCount;
		tilt = true;
		for(int i = 0; i < 7; i++)
		{
			servoValuesCurrent[i] = microPairsSlope[i][0];

		}// end for servo zero
		tiltCountdown--;
		if(tiltCountdown <= 0) tiltCountdown = 0;
	}// end if tilting

	if(tilt)
    {

        if (abs(kalAngle[X_]) < PITCH_MAX && abs(kalAngle[Y_]) < ROLL_MAX && !tiltCountdown)
        {
        	roll_error_prior = 0;   //reset PID
        	roll_integral = 0;      //reset PID
        	pitch_error_prior = 0;  //reset PID
        	pitch_integral = 0;     //reset PID
        	tilt = false;
        }// end remove tilt
    } // end if tilt
	else
	{
		#if READING_RX
		printf("Running Rotation Matrix:\n");
		#endif
		//Geometric Variables
        cRoll       = cos(rollOutput);
        sRoll       = sin(rollOutput);
        cPitch      = cos(pitchOutput);
        sPitch      = sin(pitchOutput);
        cYaw        = cos(yawOutput);
        sYaw        = sin(yawOutput);
        //Abreviated temporary variables for rotational matrices
        double cRcY, sPsRsY, cPsY, sPsRcY, cRsY, cPcY, rotA1, rotB1, rotC1, rotD1;
        cRcY      = cRoll*cYaw;
        sPsRsY    = sPitch*sRoll*sYaw;
        cPsY      = cPitch*sYaw;
        sPsRcY    = sPitch*sRoll*cYaw;
        cRsY      = cRoll*sYaw;
        cPcY      = cPitch*cYaw;

        rotA1     = cPsY - sPsRcY;
        rotB1     = -cPcY - sPsRsY;
        rotC1     = -rotA1;
        rotD1     = -rotB1;

        rotationE     = -cPcY*sRoll + sPitch*sYaw;
        rotationF     = sPitch*cYaw-cPsY*sRoll;

        //Generate Rotational Coefficients for each alpha
        for (int i = 0; i < ALPHA_NUM; i++)
        {
			cAlphaYaw[i]= cos(yawOutput - alpha_radians[i]);
			sAlphaYaw[i]= sin(yawOutput - alpha_radians[i]);
			rotationA[i]= sAlphaYaw[i]*rotA1 + cAlphaYaw[i]*cRcY;
			rotationB[i]= sAlphaYaw[i]*rotB1 + cAlphaYaw[i]*cRsY;
			rotationC[i]= cAlphaYaw[i]*rotC1 + sAlphaYaw[i]*cRcY;
			rotationD[i]= cAlphaYaw[i]*rotD1 + sAlphaYaw[i]*cRsY;
        }// end for
        //Find the servo angles
        double gamma, gammaDeg;   //angle solution in radians and degrees, respectively
        double deltaX, deltaY, deltaZ, sqrDeltaX, sqrDeltaY, sqrDeltaZ; //distance between points to servo
        double aleph, baytana, preAleph; //other angles necessary for calculations, in radians
        //For calculating Servo Angles when Law of Cosines doesn't work
        bool valid = true;


        //Loop to go through the possible positions
        for (int i = 0; i < 6; i++)
        {
			valid = true;
			int n, t;             //Used to convert i into index usable by rPoints
			n = i/2;
			t = i%2;
			//Apply rotation to the radial points
			rPointsAdj[n][t][X_] = rPoints[n][t][X_]*rotationA[n] + rPoints[n][t][Y_]*rotationB[n];
			rPointsAdj[n][t][Y_] = rPoints[n][t][X_]*rotationC[n] + rPoints[n][t][Y_]*rotationD[n];
			rPointsAdj[n][t][Z_] = rPoints[n][t][X_]*rotationE + rPoints[n][t][Y_]*rotationF + deltaZOutput;

          	//Find Distance Deltas
			if (t)
          	{
				deltaY = baseHalf + rPointsAdj[n][t][Y_];
				deltaX = BASE_L2 - rPointsAdj[n][t][X_];
          	} else {
            	deltaY = baseHalf - rPointsAdj[n][t][Y_];
            	deltaX = BASE_L1 - rPointsAdj[n][t][X_];
          	} // end if deltX deltaY
          	deltaZ = zBase + rPointsAdj[n][t][Z_];
          	//Create squared Variables
          	sqrDeltaX = square(deltaX);
          	sqrDeltaY = square(deltaY);
          	sqrDeltaZ = square(deltaZ);
          	//Use Law of Cosines to find missing angles
          	if(deltaY != 0) baytana = atanf(deltaZ/deltaY);
          	else baytana = M_PI_2;
          	preAleph = (sqrBArm + sqrDeltaX + sqrDeltaY + sqrDeltaZ - sqrAArm)/(2*B_ARM*sqrt(sqrDeltaZ + sqrDeltaY));

          	//if the side dimensions provide an "impossible triangle", use previously stored value
          	if (abs(preAleph) > 1) valid = false;

          	//If triangle is valid, update servo angle
          	if (valid)
          	{

           		aleph = acos(preAleph); //law of cosines
            	//Serial.println(aleph);
            	if (t)  gamma = M_PI - aleph - baytana; // gamma for servo 2 in pair
            	else    gamma = baytana - aleph; // gamma for servo 1 in pair
            	gammaDeg = radianToDegree*gamma;
            	servoGammas[i] = gammaDeg;    //Save angle in degrees
            	if(READING_RX) printf("%f ", servoGammas[i]);
			} else {

          	} // end valid if

          	servoValuesCurrent[i] = (float)servoSlopes[i]*(servoGammas[i])+microPairsSlope[i][0];
          	#if READING_RX
          	printf("%f \n", servoValuesCurrent[i]);
          	#endif
        	} // end for positions
        servoGammas[6] = radianToDegree*yawOutput;
        servoValuesCurrent[6] = servoSlopes[6]*(servoGammas[6])+microPairsSlope[6][0];
	}//end else no tilt
}//end motionTableUpdate()

//Input Filter for manual reference input
double inputFilter(int index, double input)
{
	double frac = 1.0/(double)span;
  	inputMean[index] = inputMean[index]*(1-frac) + input*frac;
  	return inputMean[index];
}//end inputFilter()

// Function to correct Yaw potentiometer curve
double yawInputCorrection(int inputRead)
{
  double a = -1050.4242, b = -0.00356356, c=1050.4242, ir;
  ir = (double)inputRead;
  return log((ir-c)/a)/b;
}// end yawInputCorrection()

int userInputToInt(char input)
{
	//casts and remove upper bytes
	return ((int)input)%16;
}//end userInputToInt()



//-------------------------------------------------------------------------------------------------------------
//Menu System, User Interface

void userInterface()
{
	int previousPage = menuPage;
	bool finalSelection = false;
	int intInput;
	double minutesDuration, secondsDuration, inRoll, inPitch, inYaw, inZ;
	double maxRoll = ROLL_MAX, maxPitch = PITCH_MAX, maxYaw = YAW_MAX, maxZ = DELTAZ_MAX;
	bool pause = true;
	do {
		pause = true;
		switch(menuPage) {
			case 0: //Start Menu
				do {
					menuStartText();
    				cin >> userInput;
    				validInput = inputValid(menusSelectionValidIns, 4, userInput);
    			} while (!validInput);
    			intInput = userInputToInt(userInput);
    			previousPage = menuPage;
    			menuPage = intInput;
    			break;

			case 1: //Simulation Menu
				do {
    				menuSimulationText();
    				cin >> userInput;
    				validInput = inputValid(menusSelectionValidIns, 6, userInput);
	    		} while (!validInput);
	    		intInput = userInputToInt(userInput);

				switch(intInput)
				{
					case 1: //Change Input to Boat Controller
						simulationInputType = BOAT_CONTROLLER_INPUT;
						menuPage = 1;
						break;

					case 2: //Change Input joysticks
						simulationInputType = JOYSTICK_INPUT;
						menuPage = 1;
						break;

					case 3: //Change Duration
						printf("\n\nEnter number of minutes (%dmin Maximum): ", (int)(maximumDuration/(60*1000)));
						cin >> minutesDuration;
						printf("\n\nEnter number of seconds (%dmin Maximum): ", (int)(maximumDuration/(60*1000)));
						cin >> secondsDuration;
						if(60*minutesDuration + secondsDuration > maximumDuration/1000) motionDurationMillis = maximumDuration;
						else motionDurationMillis = minutesDuration*60*1000 + secondsDuration*1000;
						menuPage = 1;
						break;

					case 4: //Toggle Logging Status
						dataLoggingEnabled = !dataLoggingEnabled;
						menuPage = 1;
						break;

					case 5: //Run Simulation
						system("clear");
						programState = SIMULATION;				//Put program into simulation mode, to be read by runMotionTable()
						printf("\nRunning Simulation\n");
						//Running
						runMotionTable();
						printf("\n\nSimulation Complete!\n\nContinue y/n?");
						while(pause)
						{
							cin >> userInput;
							if(userInput == 'y'||userInput == 'Y') pause = false;
							if(userInput == 'n'||userInput == 'N')
							{
								pause = false;
								finalSelection = true;
							}
						}//end pause while

						menuPage = 1;
						break;

					case 6: //back to tMain menu
						menuPage = 0;
						break;
				}//end switch
				break;

    		case 2: //Manual Operation Mode
    			do {
    				menuManualModeText();
    				cin >> userInput;
    				validInput = inputValid(menusSelectionValidIns, 6, userInput);
    				} while (!validInput);
    				intInput = userInputToInt(userInput);
    				switch(intInput)
    				{
	    				case 1: //Angle Hold
		    				manualModeType = MANUAL_ANGLE_HOLD;
		    				printf("\n\nEnter number Roll Angle (%ddeg Maximum): ", (int)(maxRoll));
		    				printf("Previously: %f \n",holdAngles[ROLL] );
							cin >> inRoll;
							holdAngles[ROLL] = inRoll;
							if(inRoll>maxRoll) holdAngles[ROLL] = maxRoll;
							if(inRoll<-maxRoll) holdAngles[ROLL] = -maxRoll;

							printf("\n\nEnter number Pitch Angle (%ddeg Maximum): ", (int)(maxPitch));
							printf("Previously: %f \n",holdAngles[PITCH] );
							cin >> inPitch;
							holdAngles[PITCH] = inPitch;
							if(inPitch>maxPitch) holdAngles[PITCH] = maxPitch;
							if(inPitch<-maxPitch) holdAngles[PITCH] = -maxPitch;

							printf("\n\nEnter number Yaw Angle (%ddeg Maximum): ", (int)(maxYaw));
							printf("Previously: %f \n",holdAngles[YAW] );
							cin >> inYaw;
							holdAngles[YAW] = inYaw;
							if(inYaw>maxYaw) holdAngles[YAW] = maxYaw;
							if(inYaw<-maxYaw) holdAngles[YAW] = -maxYaw;

							printf("\n\nEnter number z displacement (%d in mm): ", (int)(maxZ));
							printf("Previously: %f \n",holdAngles[DELTAZ] );
							cin >> inZ;
							holdAngles[DELTAZ] = inZ;
							if(inZ>maxZ) holdAngles[DELTAZ] = maxZ;
							if(inZ<-maxZ) holdAngles[DELTAZ] = -maxZ;

	    					menuPage = 2;
    						break;

    					case 2: //Joystick Control
		    				manualModeType = MANUAL_JOYSTICK_INPUT;
		    				menuPage = 2;
		    				break;

	    				case 3: //Change Duration
							printf("\n\nEnter number of minutes (%dmin Maximum): ", (int)(maximumDuration/(60*1000)));
							cin >> minutesDuration;
							printf("\n\nEnter number of seconds (%dmin Maximum): ", (int)(maximumDuration/(60*1000)));
							cin >> secondsDuration;
							if(60*minutesDuration + secondsDuration > maximumDuration/1000) motionDurationMillis = maximumDuration;
							else motionDurationMillis = minutesDuration*60*1000 + secondsDuration*1000;
							menuPage = 2;
							break;

						case 4: //Run Table
							system("clear");
							printf("\nRunning Motion Table Manual Mode: \n");
							printf("Manual Mode: ");
							if(manualModeType == MANUAL_ANGLE_HOLD) printf("Angle Hold at:\nRoll %f, Pitch %f, Yaw %f, z %f\n",
								holdAngles[ROLL],holdAngles[PITCH],holdAngles[YAW],holdAngles[DELTAZ] );
							if(manualModeType == MANUAL_JOYSTICK_INPUT) printf("Joystick Control\n");
							programState = MANUAL;				//Put program into manual mode, to be read by runMotionTable()
							runMotionTable();
							printf("\n\nRun Complete!\n\nContinue y/n?");
							while(pause)
							{
								cin >> userInput;
								if(userInput == 'y'||userInput == 'Y') pause = false;
								if(userInput == 'n'||userInput == 'N')
								{
									pause = false;
									finalSelection = true;
								}
							}//end pause while
							menuPage = 2;
						break;

						case 5: //back to Main menu
							menuPage = 0;
							break;
    				}//end switch
    			break;

    		case 3: //Settings
    			do {
    				menuSettingsText();
    				cin >> userInput;
    				validInput = inputValid(menusSelectionValidIns, 4, userInput);
	    			} while (!validInput);
	    			intInput = userInputToInt(userInput);
	    			printf("\nChanges will not be saved after this session ends.\n");
	    			switch(intInput)
	    			{
	    				case 1: //Proportional Gain
		    				printf("Current Proportional Gain: %f\nEnter new value:", Kp );
		    				cin >> Kp;
		    				menuPage = 3;
		    				break;

	    				case 2: //Integral Gain
		    				printf("Current Integral Gain: %f\nEnter new value:", Ki );
		    				cin >> Ki;
		    				menuPage = 3;
		    				break;

	    				case 3: //Derivative Gain
		    				printf("Current Derivative Gain: %f\nEnter new value:", Kd );
		    				cin >> Kd;
		    				menuPage = 3;
		    				break;

	    				case 4: //back tto main menu
		    				menuPage = 0;
		    				break;
	    			}//end settings swtich
    			break;

    		case 4:  //Exit
    			finalSelection = true;
    			break;

    		default:
    			menuPage = 0;
    			break;
		}// end switch case
	} while (!finalSelection);
}//end userInterface()


//Check if input is a valid input
bool inputValid(char valid[],int validSize, char input)
{
	bool validity = false;
	for(int i = 0; i < validSize; i++)
	{
	 	if(input == valid[i]) validity = true;
	}//end validity check
	return validity;
}//end inputValid()


//Menu Text for first Page
void menuStartText()
{
	system("clear");
	printf("-------------------------------Virtual Solar Boat-----------------------------\n\n\n");
	printf("Please make a numerical selection: \n");
	printf("1. Simulation Mode \n2. Manual Operation\n3. Settings \n4. Exit to Raspbian\n\n");
	printf("Selection:");
}//end menuStartText()

//Menu Text for simulation Page
void menuSimulationText()
{
	system("clear");
	printf("-------------------------------Simulation Mode--------------------------------\n\n\n");
	printf("Input Type: ");
	if(simulationInputType == BOAT_CONTROLLER_INPUT) printf("Boat Controller Input\n");
	if(simulationInputType == JOYSTICK_INPUT) printf("Manual Joystick Input\n");
	printf("Data Logging: ");
	if(dataLoggingEnabled) printf("Enabled\n");
	else printf("Disabled\n");
	printf("\n\n");
	printf("Please make a numerical selection: \n");
	printf("1. Input from controller \n");
	printf("2. Input from joysticks\n3. Duration: %f seconds\n4. Toggle Data Logging\n5. Run Table\n6. Back\n\n", motionDurationMillis/1000 );
	printf("Selection:");
}//end menuStartText()

//Menu Text for simulation Page
void menuManualModeText()
{
	system("clear");
	printf("-------------------------------Manual Operation Mode--------------------------\n\n\n");
	printf("Manual Mode: ");
	if(manualModeType == MANUAL_ANGLE_HOLD) printf("Angle Hold at:\nRoll %f, Pitch %f, Yaw %f, z %f\n",
		holdAngles[ROLL],holdAngles[PITCH],holdAngles[YAW],holdAngles[DELTAZ] );
	if(manualModeType == MANUAL_JOYSTICK_INPUT) printf("Joystick Control\n");
	printf("\n\n");
	printf("Please make a numerical selection: \n");
	printf("1. Angle Hold \n2. Joystick Control\n3. Duration: %f seconds\n4. Run Table\n5. Back\n\n", motionDurationMillis/1000 );
	printf("Selection:");
}//end menuStartText()

//Menu Text for Settings
void menuSettingsText()
{
	system("clear");
	printf("------------------------------------Settings---------------------------------\n\n\n");
	printf("Settings: ");
	printf("\n\n");
	printf("Please make a numerical selection: \n");
	printf("1. Proportional Gain: %f \n2. Integral Gain: %f \n3. Derivative Gain: %f \n4. Back\n\n", Kp, Ki, Kd );
	printf("Selection:");
}//end menuSettingsText()

//Run motion table algorithms, activate servos, calculate kinematics
void runMotionTable()
{
	//Function Variable Declarations
	unsigned int endTime, nextTurn, lastTurn;
	int cycleCount = 0, totalCycleAmount = 0;
	double meanCycleTime = 0;
	bool spiEnabled = true;
	int  startDelay = 1000000;     //Microseconds to give to position servos
	tilt = false;
	int  kalCounter = 0, kalFraction = 30; //for displaying kalAngles every tenth turn
	//Reset PID memory
	roll_error_prior = 0;  //previous diff between ref and actual roll
	roll_integral = 0;   //previous roll integral
	pitch_error_prior = 0;  //previous diff between ref and actual roll
	pitch_integral = 0;   //previous roll integral

	//Reset Values
	for (int i = 0; i < 12; ++i)
	{
		if(i<5)inputMean[i] = 0;
		if(i<7)servoGammas[i]=0;
		if(i<7)servoValuesCurrent[i] = servoZeroValues[i];
		if(programState == SIMULATION)xState[i] = x_0[i];
	}//end reset values loop

	if(spiEnabled) spiTransaction();    //Zero Servos before begining
	usleep(startDelay); 					//Delay before running updates
	endTime += startDelay/1000;				//ajust end time milliseconds
	//Timer start stuff here
	endTime = millis() + motionDurationMillis;  //when run should end
	nextTurn = micros();
	do
	{
		//Timer period stuff here
		if(micros() >= nextTurn)// time to do the things
		{
			//Timer period start stuff here
			cycleCount++;
			lastTurn = nextTurn;
			nextTurn = micros() + samplePeriodMicros;
			totalCycleAmount += (nextTurn - lastTurn);
			dt = (float)(nextTurn - lastTurn)/1000000;

			if(spiEnabled) spiTransaction();
			//if(spiEnabled) spiExchange();
			//Input Code
			if(programState == SIMULATION)
			{
				//Funtion to update boat model system state inputs *here*

				boatModelUpdate();
				conditionedInputs[ROLL] = xState[PHI_VAR]*radianToDegree;   	//Input Roll From Simulation Data
				conditionedInputs[PITCH] = xState[THETA_VAR]*radianToDegree;   //Input Pitch From Simulation Data
				conditionedInputs[YAW] = xState[PSI_VAR]*radianToDegree;   //Input Yaw From Simulation Data
				conditionedInputs[DELTAZ] = xState[Z_VAR]*1.0;   //Input deltaZ From Simulation Data, converted from m to mm
				if(dataLoggingEnabled)
				{
					//data logging code
				}// end if data logging


			}// end if references from simulation
			if(programState == MANUAL)
			{
				switch(manualModeType)
				{
					case MANUAL_ANGLE_HOLD:
					conditionedInputs[ROLL]   = holdAngles[ROLL];
					conditionedInputs[PITCH]  = holdAngles[PITCH];
					conditionedInputs[YAW]    = holdAngles[YAW];
					conditionedInputs[DELTAZ] = holdAngles[DELTAZ];
					break;

					case MANUAL_JOYSTICK_INPUT:
					conditionedInputs[ROLL]   = inputFilter(ROLL, (double)rawInputs[ROLL]*rollInputSlope + rollInputIntercept);
					conditionedInputs[PITCH]   = inputFilter(PITCH, (double)rawInputs[PITCH]*pitchInputSlope + pitchInputIntercept);
					conditionedInputs[YAW]   = inputFilter(YAW, yawInputCorrection(rawInputs[YAW])*yawInputSlope + yawInputIntercept);
					conditionedInputs[DELTAZ]   = inputFilter(DELTAZ, (double)rawInputs[DELTAZ]*deltaZInputSlope + deltaZInputIntercept);
					#if READING_RX
					for(int i = 0; i< 4; i++){printf("%f ",conditionedInputs[i] );}
					printf("\n");
					#endif
					break;
				}//end switch
			}// end if manual operation
			kalCounter = (kalCounter + 1)%kalFraction;
			if(!kalCounter) printf("Roll: %f \t Pitch: %f\n", kalAngle[Y_], kalAngle[X_]);
			motionTableUpdate();

			#if READING_RX
			printf("\n");
			for (int i = 0; i < 7; i++)
			{
				printf("%f ", servoValuesCurrent[i] );
			}//end for printing servo
			printf("\n");
			#endif

		}//end if time to do the things
	}while((millis()<endTime)&& !tilt); //durationn has not expired, or table not tilted
	if(tilt) printf("\nMotion Table Exceeded Safe Limits\n");
	else printf("\nRun Time has ended\n");
	//"Zero" out Servo Values
	for (int i=0; i<7; i++)
	{
		servoValuesCurrent[i] = servoZeroValues[i];
	}//end zeroing for
	//if(spiEnabled) spiExchange();
	if(spiEnabled) spiTransaction();
	meanCycleTime = (double)totalCycleAmount/(double)cycleCount;
	printf("\nCycles: %d\n",cycleCount );
	printf("\nMean Cycle Period: %f us\n",meanCycleTime );
	printf("\nMean Frequency: %f Hz\n",1/(meanCycleTime/1000000) );
}//end runMotionTable()

//-------------------------------------------------------------------------------------------------------------
//SPI Communication Functions

//Based on source functions:
/**********************************************************
spiTxRx
 Transmits one byte via the SPI device, and returns one byte
 as the result.
 Establishes a data structure, spi_ioc_transfer as defined
 by spidev.h and loads the various members to pass the data
 and configuration parameters to the SPI device via IOCTL
Reference: http://robotics.hobbizine.com/raspiduino.html
***********************************************************/
void spiInitializatize()
{
   fd = open("/dev/spidev0.0", O_RDWR);
   unsigned int speed = 2000000; //Found that 4Mhz is what the Arduino handles best, left at 1Mhz for now
   ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
   for(int i = 0; i < 30; i++)
   {
   	dataToTXRX[i].f = 0.0;
   }
}//end spiInitialization()

void spiTransaction ()
{


	//Handshake sequence for inputs
	//This needs to be before the receive functions for inputs
    do
    {
      ack = false;

      spiTxRx('c');  //sends a c and waits to receive a d from slave
      usleep (DEEP_SLEEP);   //delay between exchanges

      receivedByte = spiTxRx(0);
      if (receivedByte == 'd') ack = true;   //slave is ready
      usleep (DEEP_SLEEP);   //delay between exchanges
      #if READING_RX
      printf("1:  %d \n", receivedByte);
      #endif
    } while (ack == false); //end of do...while handshake

    // receive the analog inputs from Arduino slave
    rawInputs[ROLL]   = receive();
    rawInputs[PITCH]  = receive();
    rawInputs[YAW] 	  = receive();
    rawInputs[DELTAZ] = receive();
    rawInputs[THRUST] = receive();

    #if READING_RX
    cout << "ROLL   = " <<  rawInputs[ROLL] << endl;
    cout << "PITCH  = " <<  rawInputs[PITCH] << endl;
    cout << "YAW    = " <<  rawInputs[YAW] << endl;
    cout << "DELTAZ = " <<  rawInputs[DELTAZ] << endl;
    cout << "THRUST = " <<  rawInputs[THRUST] << endl;
    #endif
    //Handshake sequence for kalman
    //This needs to be before the receive functions for kalman

    do
    {
     ack = false;
     spiTxRx('e');  //sends an e and waits to receive an f from slave
     usleep (DEEP_SLEEP);   //delay between exchanges
     receivedByte = spiTxRx(0);
     if (receivedByte == 'f')  ack = true;   //slave is ready
     usleep (DEEP_SLEEP);   //delay between exchanges
     #if READING_RX
     printf("2:  %d \n", receivedByte);
     #endif
    }
    while (ack == false); //end of doWhile handshake

   // receive the analog inputs from Arduino slave
   kalAngle[Y_] = receive();
   kalAngle[X_] = receive();

   #if READING_RX
   cout << "Kalman X = " <<  kalAngle[X_] << endl;
   cout <<"Kalman Y = " <<  kalAngle[Y_] << endl;
   #endif
   //Send servo microsecond values to Arduino slave
   sendpwm(servoValuesCurrent);
   //sleep(1);

   //end transaction
}//end spiTransaction()

//SPI transmit Receive byte
int spiTxRx(char txDat)
{

  char rxDat;

  struct spi_ioc_transfer spi;

  memset (&spi, 0, sizeof (spi));

  spi.tx_buf        = (signed long)&txDat;
  spi.rx_buf        = (signed long)&rxDat;
  spi.len           = 1;

  ioctl (fd, SPI_IOC_MESSAGE(1), &spi);

  return rxDat;
}//end spiTxRx()

//Function to send servo microsecond values to Arduino slave
float sendpwm(float servoPWM[7])
{

	//char receivedByte;       //Byte received from slave
	//bool ack;                //boolean to acknowledge if slave returned handshake

	//Union to convert bytes to float and viceversa
	union one { float f; char  b [4]; } data[7];

	//store pwm from each servo in union
	for (int i = 0; i < 7; i++)
	{
		data[i].f = servoPWM[i];
	}//end for


	//pwm handshake and send pwm

	do
	{
		ack = false;

		spiTxRx('a');   //sends an a and waits to receive a b from slave
		usleep (SLEEP);

		receivedByte = spiTxRx(data[0].b[0]);
		if (receivedByte == 'b') ack = true;   //Slave is ready
		usleep (SLEEP);

	} while (!ack); // end do...while

	for(int i = 1; i < 4; i++)
	{
		spiTxRx(data[0].b[i]);   //SPI function to Transmit and Receive
		usleep (SLEEP);
	}//end send first pwm for

	//send the rest fo the values
	for (int i = 1; i < 7; i++)
	{
		for (int n = 0; n < 4; n++)
		{
			spiTxRx(data[i].b[n]);   //SPI function to Transmit and Receive
			usleep (SLEEP);
		}//end PWM bytes for
	}//end PWMs for

	return 0;
}//end sendpwm()

//Function to receive and process bytes
float receive()
{

	//union to store as bytes
	union two
	{
		float f;
		char  b [4];
	} data8;

	//Push zeros through so the Arduino can return the results
	receivedByte = spiTxRx(0);  //transmit a 0 and store received byte
	data8.b[0] = receivedByte;
	usleep (SLEEP);

	receivedByte = spiTxRx(0);  //transmit a 0 and store received byte
	data8.b[1] = receivedByte;
	usleep (SLEEP);

	receivedByte = spiTxRx(0);  //transmit a 0 and store received byte
	data8.b[2] = receivedByte;
	usleep (SLEEP);

	receivedByte = spiTxRx(0);  //transmit a 0 and store received byte
	data8.b[3] = receivedByte;

	return data8.f;             //Return resulting float
}//end receive()

void spiExchange()
{
	//PWM Update
	for(int i = 0; i < 7; i++)
	{
		dataToTXRX[SERVO_INDEX_OFFSET + i].f = servoValuesCurrent[i];
	}//end pwm update for

	digitalWrite(CS0, LOW);
	usleep(SLEEP);
	for(int i = 0; i < 14; i++)
	{
		spiTxRx(i);
		usleep(SLEEP);
		for(int n = 0; n < 4; n++)
		{
			dataToTXRX[i].b[n] = spiTxRx(dataToTXRX[i].b[n]);
			usleep(SLEEP);
		}
	}
	digitalWrite(CS0, HIGH);

	rawInputs[ROLL] = dataToTXRX[ROLL].f;
	rawInputs[PITCH] = dataToTXRX[PITCH].f;
	rawInputs[YAW] = dataToTXRX[YAW].f;
	rawInputs[DELTAZ] = dataToTXRX[DELTAZ].f;
	rawInputs[THRUST] = dataToTXRX[THRUST].f;
	kalAngle[X_] = dataToTXRX[KAL_X].f;
	kalAngle[Y_] = dataToTXRX[KAL_Y].f;
}//end spiExchange();
