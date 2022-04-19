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

/**
 * encoder data
 * init all encoder data to 0 since no distance has been traveled
 */

// timers 
unsigned long encodersT1, encodersT2;
const unsigned long ENCODER_PERIOD = 20UL;

// encoder counts
long countsLeft = 0, countsRight = 0;
long prevLeft = 0, prevRight = 0;

// distance traveled by wheel in cm
float sL = 0.0f, sR = 0.0f;

// container to store the previous distance traveled
float prevSL = 0.0f, prevSR = 0.0f;

// difference between current and previous distance traveled
float sL_Delta = 0.0f, sR_Delta = 0.0f;

// TODO: make calculation a const
// wheel and encoder constants
const float CLICKS_PER_ROTATION = 12.0f;
const float GEAR_RATIO = 75.81f;
const float WHEEL_DIAMETER = 3.2f; 
const float WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * PI; // TODO remover circumf and make one formula

// distance between the 2 drive wheels from the center point of the contact patches
const float B = 8.5f;
//TEST const float B = 8.255f;

/** 
 * position data 
 * init position objects to start at (0,0)
 */

// distance before applying dampening break force 
const float DAMPEN_RANGE = 20.0f;

// position cartesian coordinates
float x = 0.0f, y = 0.0f; 

// angle robot is facing in radians
float theta = 0.0f;

// change in position between last 2 intervals
float xDelta = 0.0f, yDelta = 0.0f, thetaDelta = 0.0f;

// change in distance traveled between last 2 intervals
float positionDelta = 0.0f;

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
float startGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));

// current linear distance from goal. Updated on motor period
float currentGoalDistance = startGoalDistance;

/* motor data */

// TODO throttle
/**
 * explore -40 to allow sharp turns based on magnitude of error
 * If very close to goal, turn sharper like a tank turn, setSpeeds(-40, 40)
 * -1% would be -MIN, -100% would be -MAX
 * 0% would be off (idle) 0
 * 1% would be +MIN, 100% would be +MAX
 * Base speed would be +50%, can be offset by a large error
 * A larger error will reduce linear speed and increase rotational speed
 */

// speed limits
const float MOTOR_MIN_SPEED = 40.0f, MOTOR_MAX_SPEED = 150.0f;

// speed constants
const float MOTOR_BASE_SPEED = 75.0f, MOTOR_IDLE = 0.0f;

// wheelSpeed containers. Set by PID output
float leftSpeed = MOTOR_IDLE, rightSpeed = MOTOR_IDLE;

// timers
unsigned long motorT1, motorT2;
const unsigned long MOTOR_PERIOD = 20UL;

/* PID data */

// TODO use rounding to smooth PID corrections and errors

// TUNE KP lower
// proportional error factor
const float KP = 100.0f;

// suggested PID correction object
float PIDCorrection = 0.0f;

// current theta vs theta of goal. Derived from arctan
float currentError = 0.0f;

/* debugging switches */

bool bEncoderDebug = false;
bool bPositionDebug = true;
bool bMotorDebug = false;
bool bPIDDebug = true;

void setup()
{
  Serial.begin(9600);
  delay(1000);
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
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // update the distance traveled by each wheel
    sL += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);
    sR += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);

    // get change of current and previous distance traveled
    sL_Delta = sL - prevSL;
    sR_Delta = sR - prevSR;

    // write previous encoder count
    prevLeft = countsLeft;
    prevRight = countsRight;

    // write previous distance traveled
    prevSL = sL;
    prevSR = sR;

    // reset timer
    encodersT2 = encodersT1;

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
  positionDelta = (sL_Delta + sR_Delta) / 2.0f;
  thetaDelta = (sR_Delta - sL_Delta) / B;

  // get polar coordinates of x and y
  xDelta = positionDelta * cos(theta + thetaDelta / 2.0f);
  yDelta = positionDelta * sin(theta + thetaDelta / 2.0f);

  // update coordinates
  x += xDelta;
  y += yDelta;
  theta += thetaDelta;

  // calculate linear distance to goal using updated position
  currentGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));

  if (bPositionDebug) debugPosition();

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
  currentError = theta - atan2(yGoal - y, xGoal - x);

  /*
  if (currentError >= 0.10f || currentError >= -0.10f)
  {
    currentError = 0.0f;
  }
  */

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
  if ((xGoal - goalPrecision <= x && xGoal + goalPrecision >= x) && (yGoal - goalPrecision <= y && yGoal + goalPrecision >= y))
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
    startGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));

    // sleep after returning home
    if (currentGoal == NUMBER_OF_GOALS)
    {
      motors.setSpeeds(0, 0);
      ledGreen(1);
    }
  }
}

/**
 * set motor speeds with PID input
 * adjusts wheel speeds individually to controll angle based on error
 * @param none reads PID result of angle correction
 * @returns void. sets left and right global wheelspeeds.
 */
void setMotors()
{
  motorT1 = millis();

  if (motorT1 > motorT2 + MOTOR_PERIOD)
  {

    leftSpeed = angleController(PIDCorrection, 1);
    rightSpeed = angleController(PIDCorrection, -1);

    motors.setSpeeds(leftSpeed, rightSpeed);

    if (bMotorDebug) debugMotors();

    motorT2 = motorT1;
  }
}

/**
 * set each wheel individually and apply a dampen force
 * @param speedInput PID controller correction
 * @param polarity positive or negative to adjust direction
 * @returns wheelSpeed speed for the desired wheel
 */
float angleController(float thetaCorrection, int polarity)
{
  // get a theta corrected wheelSpeed
  float wheelSpeed = MOTOR_BASE_SPEED + (thetaCorrection * polarity);

  // reduce wheelspeed if within threshold
  if (currentGoalDistance <= DAMPEN_RANGE)
    wheelSpeed *= (currentGoalDistance / DAMPEN_RANGE);

  // check max and min speed limits
  if (wheelSpeed < MOTOR_MIN_SPEED) wheelSpeed = MOTOR_MIN_SPEED;
  else if (wheelSpeed > MOTOR_MAX_SPEED) wheelSpeed = MOTOR_MAX_SPEED;

  return wheelSpeed;
}

void debugPID()
{
  Serial.print("PID ");
  Serial.print("err: ");
  Serial.print(currentError);
  Serial.print(" output: ");
  Serial.println(PIDCorrection);
}

void debugEncoders()
{
  Serial.print("ENC ");
  Serial.print(sL);
  Serial.print(", ");
  Serial.print(sR);

  Serial.print(" sDeltas: ");
  Serial.print(sL_Delta);
  Serial.print(", ");
  Serial.print(sR_Delta);
  Serial.print(" TraveledDist: ");
  Serial.println(positionDelta);
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
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(theta);
  Serial.print(")");
  Serial.print(" tgt (");
  Serial.print(xGoal);
  Serial.print(", ");
  Serial.print(yGoal);
  Serial.print(")");
  Serial.print(" goalDist: ");
  Serial.println(currentGoalDistance);

  /* uncomment if needed
  Serial.print(" deltas: (");
  Serial.print(xChange);
  Serial.print(", ");
  Serial.print(yChange);
  Serial.print(", ");
  Serial.print(thetaChange);
  Serial.print(")");
  */
}