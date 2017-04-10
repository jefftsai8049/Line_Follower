/*
 * NTU BIME Principles and Applications of Microcontrollers-Mechatronics (1) Yan-Fu Kuo
 * 
 * Term project - Line follower
 * 
 * Created 2016-02-04 by Ching-Wei Tsai
 * Modified 2016-03-22 by Ching-Wei Tsai
 * Modified 2016-05-03 by Kai-Jhong Huang
 * Modified 2017-04-10 by Ching-Wei Tsai
 *
 * Modified log:
 *   1. void normalize() : Average with last val, smooth the sensor val.
 *   2. turn weighting
 *   3. motor filter
 *
 * Design and build a line following robot.
 * The robot needs to have a function that detect and bypass obstacles on its path automatically.
 * The robot returns to its path of the black line after bypassing the obstacles.
 * 
 * Required Material:
 * Wheel robot set x1
 * H-bridge board x1
 * IR Module x3
 * 
 */

/* Header files */
#include "Timer.h"

/* For debug message*/
#define DEBUG_MODE 0

/* Pins defines */
#define ResistorKp   14 //for A0
#define ResistorTi   15 //for A1
#define ResistorTd   16 //for A2
#define IRSensorPinL 17 //for A3
#define IRSensorPinC 18 //for A4
#define IRSensorPinR 19 //for A5

/* Setting defines */
#define defaultSpeed 80
#define defaultKp 0.12
#define defaultTi 60 //ms
#define defaultTd 15
#define IRThreshold 0.1
#define Kp_MAX 0.5
#define Kp_MIN 0
#define Ti_MAX 300
#define Ti_MIN 30
#define Td_MAX 50
#define Td_MIN 0

enum IR_STATE
{
  IRSTATE_ERROR,
  IRSTATE_BLACK,
  IRSTATE_WHITE,
  IRSTATE_L,
  IRSTATE_LM,
  IRSTATE_M,
  IRSTATE_MR,
  IRSTATE_R
};

/* Pin map setting*/
static char motorIn[4] = {5,6,10,11};
static float IRVal[3];
Timer sensorClock;

/*
Purpose : Drive all moters
Input   : speedL - Left wheel speed (-100~100)
          speedR - Right wheel speed (-100~100)
Ouput   : N/A
Return  : N/A
*/
void moveMotor(int speedL,int speedR) {
  /* Speed range -100~+100 */
  int motorSpeed[4];

  /* Left wheel speed asign */
  speedL = (speedL > MAX_FORWARD_SPEED)? MAX_FORWARD_SPEED : speedL;
  speedL = (speedL < MAX_BACKWARD_SPEED)? MAX_BACKWARD_SPEED : speedL;
  motorSpeed[0] = (speedL >= 0)? 0 : MOTOR_OUTPUT_SCALE*(-speedL);
  motorSpeed[1] = (speedL >= 0)? MOTOR_OUTPUT_SCALE*speedL : 0;

  /* Right wheel speed asign */
  speedR = (speedR > MAX_FORWARD_SPEED)? MAX_FORWARD_SPEED : speedR;
  speedR = (speedR < MAX_BACKWARD_SPEED)? MAX_BACKWARD_SPEED : speedR;
  motorSpeed[2] = (speedR >= 0) ? 0 : MOTOR_OUTPUT_SCALE*(-speedR);
  motorSpeed[3] = (speedR >= 0) ? MOTOR_OUTPUT_SCALE*speedR : 0;

  /* Output to motor */
  for(int i = 0; i < 4; i++) {
    analogWrite(motorIn[i],motorSpeed[i]);
  }
}

/*
Purpose : Read IR sensor value
Input   : N/A
Ouput   : val - Pointer for IR sensor values
Return  : N/A
*/
void getIRStatus() {
	IRVal[0] = analogRead(IRSensorPinL);
	IRVal[1] = analogRead(IRSensorPinC);
	IRVal[2] = analogRead(IRSensorPinR);
}

/*
Purpose : Normalize the IR sensor value
Input   : N/A
Ouput   : val - Pointer for IR sensor values
Return  : N/A
*/
void getIRNormalize(float* val) {

  /*
    This part of value is measure by experiment.
    All of the IR sensors have different output range and feature.
  */

  /* Each IR sensor max and min */
  static float maxVal[3] = {600.0,600.0,600.0};
  static float minVal[3] = {250.0,250.0,250.0};
  static float lastVal[3] = {425.0,425.0,425.0};

  for(int i = 0; i < 3 ;i++) {
    /* Average with last val, val=[v(0)+v(-1)]/2 */
    val[i]=(IRVal[i]+lastVal[i])/2;
    lastVal[i]=val[i];
    
    /* For update sensor max and min */
    if(val[i] > maxVal[i])
      maxVal[i] = val[i];
    if(val[i] < minVal[i])
      minVal[i] = val[i];

    /* Linearize the IR sensor value */
    /* 
      ax^3+bx^2+cx+d = y
      The a,b and c value is calculated from linear regression
    */
    val[i] = (val[i]-minVal[i])/(maxVal[i]-minVal[i]);
    val[i] =1.8704*val[i]*val[i]*val[i]-2.4402*val[i]*val[i]+1.5652*val[i]-0.0326;
  }

#ifdef DEBUG_MODE
  Serial.print("IR Value L:");
  Serial.print(val[0]);
  Serial.print("  C:");
  Serial.print(val[1]);
  Serial.print("  R:");
  Serial.print(val[2]);
#endif
}

/*
Purpose : Get Line state
Input   : val - Pointer for IR sensor values
Ouput   : N/A
Return  : line state
*/
int getLineState(float* val)
{
	bool state[3];
	for (int i = 0; i < 3; i++) {
		if(val[i]>IRThreshold)
			state[i]=true;
		else
			state[i]=false;
	}

	if     (state[0]==true&&state[1]==true&&state[2]==true)
		return IRSTATE_BLACK;
	else if(state[0]==false&&state[1]==false&&state[2]==false)
		return IRSTATE_WHITE;
	else if(state[0]==true&&state[1]==false&&state[2]==false)
		return IRSTATE_L;
	else if(state[0]==true&&state[1]==true&&state[2]==false)
		return IRSTATE_LM;
	else if(state[0]==false&&state[1]==true&&state[2]==false)
		return IRSTATE_M;
	else if(state[0]==false&&state[1]==true&&state[2]==true)
		return IRSTATE_MR;
	else if(state[0]==false&&state[1]==false&&state[2]==true)
		return IRSTATE_R;
	else
		return IRSTATE_ERROR;
}

/*
Purpose : Get Line position
Input   : val   - Pointer for IR sensor values
          state - IR state
Ouput   : N/A
Return  : line position
*/
int getLinePosition(float* val,int state)
{  
  int lineposition;
  
  if(state==IRSTATE_BLACK||state==IRSTATE_WHITE||state==IRSTATE_ERROR)
    lineposition=0;  
  else if(state==IRSTATE_L)
	  lineposition=-900+(-900+val[0]*1000);
  else if(state==IRSTATE_R)
  	lineposition=900+(900-val[2]*1000);
  else {
    /* Calculate line position */
    float IRAvg = 0;
    float IRSum = 0;
    for (int i = 0; i < 3; i++){
      /* Calculating the weighted mean */
      IRAvg += val[i] * (i-1) * 1000;   
      IRSum += val[i];
    }
    lineposition=int(IRAvg / IRSum); 
  }  
  return lineposition;
}

/*
Purpose : Arduino setup function
Input   : N/A
Ouput   : N/A
Return  : N/A
*/
void setup() {
  /* Pin mode setup */
  pinMode(ResistorKp,INPUT);
  pinMode(ResistorTi,INPUT);
  pinMode(ResistorTd,INPUT);
	pinMode(IRSensorPinL,INPUT);
	pinMode(IRSensorPinC,INPUT);
	pinMode(IRSensorPinR,INPUT);
  for(int i = 0; i < 4 ;i++) {
    pinMode(motorIn[i],OUTPUT);
  }
  
  /* Set update duration */
  sensorClock.every(1,getIRStatus);

#ifdef DEBUG_MODE
  Serial.begin(9600);
  Serial.println("Start");
#endif

  /* For devide warm up*/
  delay(200);
}

/*
Purpose : Arduino loop function
Input   : N/A
Ouput   : N/A
Return  : N/A
*/
void loop() {
  static float IRVal_normalized[3];

  /* Update clock */
  sensorClock.update();

  /* Get IR value*/
  getIRStatus();

  /* Normalize sensor value */  
  getIRNormalize(IRVal_normalized);

  /* Get IR state and line position*/
  int IRstate=getLineState(IRVal_normalized);
  int linePosition = getLinePosition(IRVal_normalized,IRstate);

  /* PID control for adjust carspeed */
  float carSpeed = defaultSpeed;
  float Kp = defaultKp;
  float Ti = defaultTi;
  float Td = defaultTd;

  static int lastTime; 
  static int oldlinePosition;
  int DX = linePosition-oldlinePosition;
  oldlinePosition = linePosition;  
  int DT = millis()-lastTime;
  lastTime = millis();
 
  float ddd = Td*DX/DT;

  float MV = Kp*(linePosition+(1/Ti)*DT*linePosition+ddd);

  float motorSpeedL = carSpeed-MV;
  float motorSpeedR = carSpeed+MV;

  moveMotor(motorSpeedL,motorSpeedR);

#ifdef DEBUG_MODE
  Serial.print("  state:");
  Serial.print(IRstate);
  Serial.print(" linePosition:");
  Serial.print(linePosition);
  Serial.print(" DT:");
  Serial.print(DT);
  Serial.print("  Kp:");
  Serial.print(Kp);
  Serial.print("  Ti:");
  Serial.print(Ti);
  Serial.print("  Td:");
  Serial.println(Td);
#endif
}



  
