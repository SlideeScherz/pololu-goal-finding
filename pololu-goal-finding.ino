/*
 Name: polulo_goal_finding.ino
 Created:	4/15/2022 12:59:32 PM
 Authors:	Thomas Diaz-Piedra, Scott Scherzer, Christopher Fioti
 Version: 1.0.1

 All distance and measurements are expressed in cm
 All trig calculations are expressed in radians
*/

#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Motors motors;
Buzzer buzzer;

// constants for parsing pos, delta, goal arrays
#define X 0
#define Y 1
#define THETA 2

/* encoder data */

// timers 
unsigned long encodersT1, encodersT2;
const unsigned long ENCODER_PERIOD = 20UL;

// encoder counts
long countsLeftT1 = 0, countsRightT1 = 0;

// container to store the previous counts
long countsLeftT2 = 0, countsRightT2 = 0;

// distance traveled by wheel in cm
float sLeftT1 = 0.0f, sRightT1 = 0.0f;

// container to store the previous distance traveled
float sLeftT2 = 0.0f, sRightT2 = 0.0f;

// difference between current and previous distance traveled
float sLeftDelta = 0.0f, sRightDelta = 0.0f;

// change in distance traveled between last 2 intervals
float sDelta = 0.0f;

/* wheel data */

// wheel and encoder constants, turtle edition
const float CLICKS_PER_ROTATION = 12.0f;
const float GEAR_RATIO = 75.81f;
const float WHEEL_DIAMETER = 3.2f;

// cm traveled each gear tick
const float DIST_PER_TICK = (WHEEL_DIAMETER * PI) / (CLICKS_PER_ROTATION * GEAR_RATIO);

// distance between the 2 drive wheels from the center point of the contact patches
const float B = 8.5f; //TUNE B

/* position data */

// positional polar coordinates
float pos[3] = { 0.0f, 0.0f, 0.0f };

// change in position between last 2 intervals
float delta[3] = { 0.0f, 0.0f, 0.0f };

/* goal data */

// index of GOAL array to select which goal to navigate to 
int currentGoal = 0;

// len of GOALS array, how many goals we want to navigate to 
const int NUMBER_OF_GOALS = 4;

// goal containers
float xGoals[NUMBER_OF_GOALS] = { 30.0f, -30.0f, 30.0f, 0.0f };
float yGoals[NUMBER_OF_GOALS] = { 15.0f, 0.0f, -15.0f, 0.0f };
float xGoal = xGoals[currentGoal];
float yGoal = yGoals[currentGoal];

// allow a slight error within this range
float goalPrecision = 0.75f;

// starting linear distance from goal. Updated on goal change
//TODO swapped ygoal and y
float startGoalDistance = sqrt(sq(xGoal - pos[X]) + sq(yGoal - pos[Y]));

// current linear distance from goal. Updated on motor period
float currentGoalDistance = startGoalDistance;

/* motor data */

// TODO throttle
/**
 * child of the dual-pid, replace angleController
 * explore -40 to allow sharp turns based on magnitude of error
 * If very close to goal, turn sharper like a tank turn, setSpeeds(-40, 40)
 * -1% would be -MIN, -100% would be -MAX
 * 0% would be off (idle) 0
 * 1% would be +MIN, 100% would be +MAX
 * Base speed would be +50%, can be offset by a large error
 * A larger error will reduce linear speed and increase rotational speed
 */

// distance before applying dampening break force 
const float DAMPEN_RANGE = 20.0f;

// speed limits
const float MOTOR_MIN_SPEED = 40.0f, MOTOR_MAX_SPEED = 150.0f;

// speed constants
const float MOTOR_BASE_SPEED = 75.0f;

// wheelSpeed containers. Set by PID output
float leftSpeed = MOTOR_MIN_SPEED, rightSpeed = MOTOR_MIN_SPEED;

// timers
unsigned long motorT1, motorT2;
const unsigned long MOTOR_PERIOD = 20UL;

/* PID data */

 // TUNE lower KP and more dynamic turning
// proportional error factor
const float KP = 100.0f;

// suggested PID correction object
float PIDCorrection = 0.0f;

// current theta vs theta of goal. Derived from arctan
float currentError = 0.0f;

// used in calculating error
float arctanToGoal = 0.0f;

/* debugging switches */

bool bEncoderDebug = false;
bool bPositionDebug = true;
bool bMotorDebug = false;
bool bPIDDebug = false;

void setup()
{
  Serial.begin(9600);
  delay(3000);
  printHeadings();
}

void loop()
{
  if (currentGoal < NUMBER_OF_GOALS)
  {
    readEncoders();
    setMotors();
    checkGoalStatus();
  }
}

/**
* Read encoder data to calculate the distance traveled
* Also, calculate the change in position to calculate position
* @returns void sets sL and sR delta for position calculation
*/
void readEncoders()
{
  encodersT1 = millis();

  if (encodersT1 > encodersT2 + ENCODER_PERIOD)
  {

    // read current encoder count
    countsLeftT1 += encoders.getCountsAndResetLeft();
    countsRightT1 += encoders.getCountsAndResetRight();

    // update the distance traveled by each wheel
    sLeftT1 += (countsLeftT1 - countsLeftT2) * DIST_PER_TICK;
    sRightT1 += (countsRightT1 - countsRightT2) * DIST_PER_TICK;

    // get change of current and previous distance traveled
    sLeftDelta = sLeftT1 - sLeftT2;
    sRightDelta = sRightT1 - sRightT2;

    // write previous encoder count
    countsLeftT2 = countsLeftT1;
    countsRightT2 = countsRightT1;

    // write previous distance traveled
    sLeftT2 = sLeftT1;
    sRightT2 = sRightT1;

    // reset timer
    encodersT2 = encodersT1;

    if (bEncoderDebug) debugEncoders();

    // send encoder data to calculate x,y,theta position
    getPosition();
  }
}

/**
 * Update the robots current position using wheel encoder data
 * @param none reads change from sL and sR
 * @returns void updates x,y, theta and currentGoalDistance
 */
void getPosition()
{
  // update position using the deltas of each
  sDelta = (sLeftDelta + sRightDelta) / 2.0f;
  delta[THETA] = (sRightDelta - sLeftDelta) / B;

  // get polar coordinates of x and y
  delta[X] = sDelta * cos(pos[THETA] + delta[THETA] / 2.0f);
  delta[Y] = sDelta * sin(pos[THETA] + delta[THETA] / 2.0f);

  // update coordinates
  pos[X] += delta[X];
  pos[Y] += delta[Y];
  pos[THETA] += delta[THETA];

  // calculate linear distance to goal using updated position
  //TODO make sure this works
  currentGoalDistance = sqrt(sq(xGoal - pos[X]) + sq(yGoal - pos[Y]));

  if (bPositionDebug) debugPosition();

  //TEMP 
  //plotPosition();

  // send position data to PID controller to get a correction
  getPIDCorrection();
}

/**
 * get a proportionate correction based on current theta vs goal
 * a positive currentError will suggest a left turn
 * a negative currentError will suggest a right turn
 * @param none. Reads position data and goal to set theta error
 * @returns void set PIDcorrection to a proportional angle correction
 */
void getPIDCorrection()
{
  arctanToGoal = atan2(yGoal - pos[Y], xGoal - pos[X]);

  currentError = pos[THETA] - arctanToGoal;

  PIDCorrection = KP * currentError;

  if (bPIDDebug) debugPID();
}

/**
 * check status of goals
 * triggered when goalCompleted is set
 * @returns void. selects next goal and resets startGoalDistance
 */
void checkGoalStatus()
{

  bool goalCompleted = false;

  // check completed goal and set status
  if ((xGoal - goalPrecision <= pos[X] && xGoal + goalPrecision >= pos[X]) && (yGoal - goalPrecision <= pos[Y] && yGoal + goalPrecision >= pos[Y]))
    goalCompleted = true;

  // advance to next goal
  if (goalCompleted)
  {
    // uncomment if you want to be annoyed
    //buzzer.play("c32");

    // cycle next goal
    currentGoal++;
    xGoal = xGoals[currentGoal];
    yGoal = yGoals[currentGoal];

    // update start goal distance
    startGoalDistance = sqrt(sq(xGoal - pos[X]) + sq(yGoal - pos[Y]));

    // sleep after returning home
    if (currentGoal == NUMBER_OF_GOALS)
    {
      motors.setSpeeds(0, 0);
      ledGreen(1);
    }
  }
}

//TODO: anglePID
//TODO: speedPID

/**
 * set motor speeds with PID input
 * @returns void. sets left and right global wheelspeeds.
 */
void setMotors()
{
  motorT1 = millis();

  if (motorT1 > motorT2 + MOTOR_PERIOD)
  {

    leftSpeed = MOTOR_BASE_SPEED + PIDCorrection;
    rightSpeed = MOTOR_BASE_SPEED + PIDCorrection * -1.0f;

    // reduce wheelspeed if within threshold
    if (currentGoalDistance <= DAMPEN_RANGE)
    {
      leftSpeed *= (currentGoalDistance / DAMPEN_RANGE);
      rightSpeed *= (currentGoalDistance / DAMPEN_RANGE);
    }
      
    // check max and min speed limits
    if (leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
    else if (leftSpeed > MOTOR_MAX_SPEED) leftSpeed = MOTOR_MAX_SPEED;

    if (rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;
    else if (rightSpeed > MOTOR_MAX_SPEED) rightSpeed = MOTOR_MAX_SPEED;

    motors.setSpeeds(leftSpeed, rightSpeed);

    if (bMotorDebug) debugMotors();

    motorT2 = motorT1;
  }
}

void debugPID()
{
  Serial.print("PID ");
  Serial.print("atan: ");
  Serial.print(arctanToGoal);
  Serial.print(" err: ");
  Serial.print(currentError);
  Serial.print(" output: ");
  Serial.println(PIDCorrection);
}

void debugEncoders()
{
  Serial.print("ENC ");
  Serial.print("countsT1: ");
  Serial.print(countsLeftT1);
  Serial.print(", ");
  Serial.print(countsRightT1);
  Serial.print(" countsT2: ");
  Serial.print(countsLeftT2);
  Serial.print(", ");
  Serial.print(countsRightT2);
  Serial.print(" sT1: ");
  Serial.print(sLeftT1);
  Serial.print(", ");
  Serial.print(sRightT1);
  Serial.print(" sT2: ");
  Serial.print(sLeftT2);
  Serial.print(", ");
  Serial.print(sRightT2);
  Serial.print(" sDeltas: ");
  Serial.print(sLeftDelta);
  Serial.print(", ");
  Serial.print(sRightDelta);
  Serial.print(" posDelta: ");
  Serial.println(sDelta);
}

void debugMotors()
{
  Serial.print("MOT ");
  Serial.print(leftSpeed);
  Serial.print(", ");
  Serial.println(rightSpeed);
}

void debugPosition()
{
  Serial.print("POS (");
  Serial.print(pos[X]);
  Serial.print(", ");
  Serial.print(pos[Y]);
  Serial.print(", ");
  Serial.print(pos[THETA]);
  Serial.print(")");
  Serial.print(" tgt (");
  Serial.print(xGoal);
  Serial.print(", ");
  Serial.print(yGoal);
  Serial.print(")");
  Serial.print(" goalDist: ");
  Serial.println(currentGoalDistance);
}

// export the log for excel plotting and tuning
void plotPosition()
{
  Serial.print(pos[X]);
  Serial.print(",");
  Serial.print(pos[Y]);
  Serial.print(",");
  Serial.print(pos[THETA]);
  Serial.print(",");
  Serial.print(xGoal);
  Serial.print(",");
  Serial.print(yGoal);
  Serial.print(",");
  Serial.print(currentGoalDistance);
  Serial.print(",");
  Serial.print(arctanToGoal);
  Serial.print(",");
  Serial.print(currentError);
  Serial.print(",");
  Serial.print(PIDCorrection);
  Serial.print(",");
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.println(rightSpeed);
}

void printHeadings()
{
  Serial.println(); // nextline
  Serial.println(__TIMESTAMP__);
  
  Serial.print("X,");
  Serial.print("Y,");
  Serial.print("Theta,");
  Serial.print("xGoal,");
  Serial.print("yGoal,");
  Serial.print("currentGoalDistance,");
  Serial.print("arctanToGoal,");
  Serial.print("currentError,");
  Serial.print("PIDCorrection,");
  Serial.print("leftSpeed,");
  Serial.println("rightSpeed,");
}